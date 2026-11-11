# 仅供生命周期集成测试的 Unix socket peer：用合成状态模拟准备、延迟解锁与下降。
# 它不使用 ROS/串口；TAKEOFF/LAND 只改变本进程字典和 trace，用于检查上层是否保持恢复心跳。
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

# 父进程传入已建立的 socket FD 与记录路径，避免绑定生产 socket 或启动 Agent。
fd, trace_arg = sys.argv[1:3]
pending_arm_fixture = sys.argv[3:] == ['pending_arm']
trace = Path(trace_arg)
conn = socket.socket(fileno=int(fd))
conn.setblocking(False)
buf = b''
state = ground()
state['kind'] = 'state'
stopping = [False]
# SIGTERM 只请求退出；若模拟飞机仍 armed 或有延迟解锁/下降，则继续输出恢复状态。
signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
descent_until = None
arm_until = None
with trace.open('w', buffering=1) as f:
    while True:
        now = time.monotonic()
        # 延迟模拟解锁完成；即使操作员已退出，仍可能迟到变为 armed，供恢复链验证。
        if arm_until and now >= arm_until:
            state.update(
                phase='DESCENT' if descent_until else 'HEIGHT_HOLD',
                armed=True,
                landed=False,
                at_rest=False,
            )
            arm_until = None
        # 模拟下降计时结束后才同时反馈接地、静止、上锁与 Stabilized。
        if descent_until and now >= descent_until:
            state.update(phase='SAFE_DISARMED', armed=False, landed=True, at_rest=True, nav_state=2)
            descent_until = None
        # 每次循环输出完整 JSON 状态行，必须由接收端处理分包/连续行。
        conn.sendall((json.dumps(state) + '\n').encode())
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
                f.write(json.dumps({'seq': int(seq), 'op': op, 'now': now}) + '\n')
                # READY 只建立模拟 Offboard 权限；不直接标记 armed。
                if op == 'ATTITUDE_READY':
                    state = {**ready(), 'kind': 'state'}
                elif op == 'TAKEOFF':
                    if pending_arm_fixture:
                        state.update(phase='REQUEST_ARM', relative_height_m=0.0)
                        arm_until = now + 0.4
                    else:
                        state.update(
                            phase='HEIGHT_HOLD',
                            armed=True,
                            landed=False,
                            at_rest=False,
                            relative_height_m=float(values[0]),
                        )
                # 待解锁期间收到 LAND 也要启动恢复下降，不能忽略尚未显现的 armed 风险。
                elif op == 'LAND' and (state['armed'] or arm_until):
                    state.update(phase='DESCENT')
                    descent_until = now + (0.8 if arm_until else 0.4)
                reply = {**state, 'kind': 'reply', 'seq': int(seq), 'accepted': True, 'message': op}
                conn.sendall((json.dumps(reply) + '\n').encode())
# 测试 peer 结束后关闭自己持有的 socket；父进程另有退出与清理断言。
conn.close()
