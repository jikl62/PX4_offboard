#!/usr/bin/env python3
# 正式导航服务测试使用的假原生进程：消费 --params-file 和继承 FD，模拟持有控制权后的优雅下降。
# 生成的 trace 记录 RC 开关、程序解锁门和日志选项，证明启动参数实际到达了子进程。
"""ROS/native boundary peer; no serial or flight effects."""
import json
from pathlib import Path
import select, signal, socket, sys, time
import yaml

config = Path(sys.argv[sys.argv.index('--params-file') + 1])
p = yaml.safe_load(config.read_text())['fc01_system']['ros__parameters']
fd = int(next(x.split('=', 1)[1] for x in sys.argv if x.startswith('--test-fd=')))
conn = socket.socket(fileno=fd)
conn.setblocking(False)
stop = [None]
signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, time.monotonic()))
buf = b''
ops = []
# 信号后保留 0.35 秒 DESCENT，再输出 SAFE_DISARMED；此延迟用于检查监督器不会过早杀进程。
while True:
    end = stop[0] is not None and time.monotonic() - stop[0] > 0.35
    state = dict(
        kind='state',
        phase='SAFE_DISARMED' if end else 'DESCENT' if stop[0] else 'HEIGHT_HOLD',
        reason='native recovery',
        status_fresh=True,
        local_fresh=True,
        land_fresh=True,
        armed=not end,
        landed=end,
    )
    conn.sendall((json.dumps(state) + '\n').encode())
    # 只有最终未解锁接地状态发送完成才退出，不能用进程活着替代飞机反馈。
    if end:
        break
    if select.select([conn], [], [], 0.04)[0]:
        data = conn.recv(4096)
        if not data:
            break
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            ops.append(line.decode().split()[1])
(config.parent / 'trace.json').write_text(
    json.dumps(
        dict(
            ops=ops,
            rc_enabled=p['rc_navigation_enabled'],
            automatic_arm_disabled=not p['arm_enabled'],
            external_log_disabled='--disable-external-lib-logs' in sys.argv,
        )
    )
)
# 明确释放测试连接，不操作真实设备或 systemd。
conn.close()
