# standby 的最小 socket 对端：正常、静默和断线三种模式均只记录收到的协议行。
# 用真实进程和 SIGTERM 检查监督器能区分“进程存活”“有新反馈”和“已优雅退出”。
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


# 信号处理器只置退出标志，循环负责写 STOPPED 和关闭连接，不在处理器中执行复杂 I/O。
def stop(*_):
    global stopping
    stopping = True


signal.signal(signal.SIGTERM, stop)
buf = b''
start = time.monotonic()
with trace.open('w', buffering=1) as f:
    while True:
        # silent 模式刻意不给任何状态，以验证旧快照不会被存活进程刷新。
        if mode != 'silent':
            state = dict(
                kind='state',
                phase='IDLE',
                status_fresh=True,
                local_fresh=True,
                land_fresh=True,
                armed=False,
                landed=True,
            )
            conn.sendall((json.dumps(state) + '\n').encode())
        if stopping:
            f.write('STOPPED\n')
            break
        # 主动断开 IPC 后仍保持进程存活，验证监督器不能把 socket EOF 当成原生安全退出。
        if mode == 'close' and time.monotonic() - start > 0.1:
            conn.close()
            while not stopping:
                time.sleep(0.02)
            f.write('STOPPED\n')
            break
        if select.select([conn], [], [], 0.03)[0]:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b'\n' in buf:
                row, buf = buf.split(b'\n', 1)
                f.write(row.decode() + '\n')
conn.close()
