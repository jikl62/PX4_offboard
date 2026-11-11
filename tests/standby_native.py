#!/usr/bin/env python3
# 待机监督器的假原生边界：真实子进程/信号/socket，状态完全由 fixture 生成。
# 解析原启动参数并记录所有动作门是否关闭，特别检查 Humble 的 csv_path 空字符串 YAML 覆盖。
"""Native process boundary fixture with real filesystem/socket and signals."""
import json
from pathlib import Path
import select
import signal
import socket
import sys
import time
import yaml

# 读取监督器实际传入的会话配置，而不是测试中另造一份参数。
config_path = Path(sys.argv[sys.argv.index('--params-file') + 1])
config = yaml.safe_load(config_path.read_text())['fc01_system']['ros__parameters']
flags = (
    'flight_enabled',
    'flight_actions_enabled',
    'arm_enabled',
    'takeoff_enabled',
    'flight_parameters_confirmed',
    'rc_takeover_confirmed',
    'autonomous_failsafe_confirmed',
    'bench_arm_only_enabled',
)
sock_path = next(x.split(':=', 1)[1] for x in sys.argv if x.startswith('socket_path:='))
csv = config.get('csv_path', '')
for value in sys.argv:
    # 空 YAML 字符串必须传成带引号的有效 RHS；零长度 RHS 在 Humble 中会被拒绝。
    if value.startswith('csv_path:='):
        rhs = value.split(':=', 1)[1]
        if not rhs:
            raise ValueError('Humble rcl_parse_yaml_value rejects zero-length override RHS')
        csv = yaml.safe_load(rhs)
        if not isinstance(csv, str):
            raise ValueError('csv_path must be a string')
fd = int(next(x.split('=', 1)[1] for x in sys.argv if x.startswith('--test-fd=')))
conn = socket.socket(fileno=fd)
conn.setblocking(False)
stopped = [False]
signal.signal(signal.SIGTERM, lambda *_: stopped.__setitem__(0, True))
ops, buf = [], b''
# 稳定发布未解锁 IDLE，直到收到 SIGTERM；测试可用 --test-no-ground 单独移除接地证据。
while True:
    state = dict(
        kind='state',
        phase='IDLE',
        status_fresh=True,
        local_fresh=True,
        land_fresh=True,
        armed=False,
        landed='--test-no-ground' not in sys.argv,
    )
    conn.sendall((json.dumps(state) + '\n').encode())
    if stopped[0]:
        break
    if select.select([conn], [], [], 0.04)[0]:
        data = conn.recv(4096)
        if not data:
            break
        buf += data
        while b'\n' in buf:
            row, buf = buf.split(b'\n', 1)
            ops.append(row.decode().split()[1])
(config_path.parent / 'native_trace.json').write_text(
    json.dumps(
        dict(
            ops=ops,
            csv_value=csv,
            external_file_logs_disabled='--disable-external-lib-logs' in sys.argv,
            all_gates_closed=all(config[x] is False for x in flags),
        )
    )
)
# 退出前写下命令及动作门证据，再释放继承连接；整个 fixture 不接触 PX4。
conn.close()
