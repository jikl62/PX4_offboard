#!/usr/bin/env python3
# R6.3 历史候选配置的离线断言：保护已审定推力数字和默认关闭的动作门。
# 这里只读文本，不加载 ROS、不接串口；断言失败直接使离线构建流程失败。
"""Fail closed if R6.3 candidate config silently enables actual flight."""
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()


# 提取独占一行的标量值；字段缺失立即终止，防止以缺省值掩盖不完整配置。
def scalar(name):
    m = re.search(r'^\s*' + re.escape(name) + r':\s*([^#\n]+)', text, re.M)
    if not m:
        raise SystemExit('MISSING_CONFIG_KEY: ' + name)
    return m.group(1).strip()


# 归一化推力单位为 0～1；浮点容差仅用于文本到浮点的数值比较，不放宽候选参数合同。
assert abs(float(scalar('aa_hover_thrust')) - 0.50) < 1e-12
assert abs(float(scalar('aa_takeoff_reserve_thrust')) - 0.04) < 1e-12
assert abs(float(scalar('aa_thrust_min')) - 0.12) < 1e-12
assert abs(float(scalar('aa_thrust_max')) - 0.80) < 1e-12
assert abs(float(scalar('aa_thrust_slew')) - 0.40) < 1e-12
# 此历史候选必须所有动作门保持 false；本工具只检查，不修改配置以求通过。
for key in (
    'flight_enabled',
    'flight_actions_enabled',
    'arm_enabled',
    'takeoff_enabled',
    'flight_parameters_confirmed',
    'bench_arm_only_enabled',
):
    assert scalar(key) == 'false', f'UNSAFE_DEFAULT: {key}={scalar(key)}'
print('R63_CONFIG_0P50_AND_DEFAULT_GATES_CLOSED_PASS')
