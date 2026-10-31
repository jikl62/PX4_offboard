"""Socket peer for standby lifecycle tests. No ROS or serial access."""
import json
from pathlib import Path
import select
import signal
import socket
import sys
import time

conn = socket.socket(fileno=int(sys.argv[1]))
trace = Path(sys.argv[2])
mode = sys.argv[3]
stopping = False

def stop(*_):
    global stopping
    stopping = True

signal.signal(signal.SIGTERM, stop)
buf = b''
start = time.monotonic()
with trace.open('w', buffering=1) as f:
    while True:
        if mode != 'silent':
            state = dict(kind='state', phase='IDLE', status_fresh=True,
                         local_fresh=True, land_fresh=True, armed=False, landed=True)
            conn.sendall((json.dumps(state)+'\n').encode())
        if stopping:
            f.write('STOPPED\n')
            break
        if mode == 'close' and time.monotonic()-start > .1:
            conn.close()
            while not stopping:
                time.sleep(.02)
            f.write('STOPPED\n')
            break
        if select.select([conn], [], [], .03)[0]:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b'\n' in buf:
                row, buf = buf.split(b'\n', 1)
                f.write(row.decode()+'\n')
conn.close()
