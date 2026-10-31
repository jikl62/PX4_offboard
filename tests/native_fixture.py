"""Test-only Unix peer: no ROS, serial device, or flight commands."""
import json
from pathlib import Path
import select
import signal
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_control import ground, ready

fd, trace_arg = sys.argv[1:3]
pending_arm_fixture = sys.argv[3:] == ['pending_arm']
trace = Path(trace_arg)
conn = socket.socket(fileno=int(fd))
conn.setblocking(False)
buf = b''
state = ground()
state['kind'] = 'state'
stopping = [False]
signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
descent_until = None
arm_until = None
with trace.open('w', buffering=1) as f:
    while True:
        now = time.monotonic()
        if arm_until and now >= arm_until:
            state.update(phase='DESCENT' if descent_until else 'HEIGHT_HOLD', armed=True,
                         landed=False, at_rest=False)
            arm_until = None
        if descent_until and now >= descent_until:
            state.update(phase='SAFE_DISARMED', armed=False, landed=True, at_rest=True, nav_state=2)
            descent_until = None
        conn.sendall((json.dumps(state)+'\n').encode())
        if stopping[0] and not state['armed'] and descent_until is None and arm_until is None:
            break
        if select.select([conn], [], [], 0.04)[0]:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                seq, op, *values = line.decode().split()
                f.write(json.dumps({'seq': int(seq), 'op': op, 'now': now})+'\n')
                if op == 'ATTITUDE_READY':
                    state = {**ready(), 'kind': 'state'}
                elif op == 'TAKEOFF':
                    if pending_arm_fixture:
                        state.update(phase='REQUEST_ARM', relative_height_m=0.0)
                        arm_until = now+0.4
                    else:
                        state.update(phase='HEIGHT_HOLD', armed=True, landed=False, at_rest=False,
                                     relative_height_m=float(values[0]))
                elif op == 'LAND' and (state['armed'] or arm_until):
                    state.update(phase='DESCENT')
                    descent_until = now+(0.8 if arm_until else 0.4)
                reply = {**state, 'kind': 'reply', 'seq': int(seq), 'accepted': True, 'message': op}
                conn.sendall((json.dumps(reply)+'\n').encode())
conn.close()
