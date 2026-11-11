#!/usr/bin/env python3
# 纯离线视觉回放入口：严格检查 JSONL，再把数据编码给本机 C++ Runtime 回放驱动。
# 不导入 ROS、不接硬件、不发布飞控消息；理想仿真与录制数据标识分开，回放结果不等于实机标定。
"""Strict JSONL -> host Runtime replay. No ROS imports or hardware endpoints."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parent
INCLUDE = ROOT / 'native/src/jikl_fc01_system/include'
# 状态是飞机实际测量，command 是目标指令；两套允许字段分别列出，未知字段直接拒绝以发现拼写/协议错误。
STATE_KEYS = {
    'sequence',
    'stamp_ns',
    'reset_counter',
    'frame_id',
    'body_frame_id',
    'source_id',
    'angular_velocity_source',
    'initialized',
    'tracking',
    'calibration_valid',
    'valid_fields',
    'position',
    'velocity',
    'attitude',
    'angular_velocity',
    'pose_covariance',
    'twist_covariance',
    'quality',
    'received_age_seconds',
}
COMMAND_KEYS = {
    'sequence',
    'stamp_ns',
    'reset_counter',
    'source_id',
    'frame_id',
    'mode',
    'valid',
    'position_valid',
    'yaw_valid',
    'sticks_valid',
    'position',
    'yaw',
    'sticks',
    'received_age_seconds',
}
MODES = {'Disabled': 0, 'Hold': 1, 'Cruise': 2, 'Follow': 3, 'Land': 4}
# 候选参数仅限此白名单，不能在数据集里夹带任意 Runtime 配置或硬件入口。
CANDIDATES = {
    'state_timeout',
    'command_timeout',
    'max_measurement_delay',
    'future_tolerance',
    'position_gain',
    'velocity_gain',
    'max_speed',
    'max_acceleration',
    'max_tilt_deg',
    'max_yaw_rate',
    'max_vertical_speed',
    'max_radius',
    'max_height',
    'min_quality',
    'max_position_jump',
    'max_velocity_jump',
}


# 区分输入合同错误与编译/进程错误，统一转换成命令行失败结果。
class InputError(ValueError):
    pass


# 每层对象先验证类型与字段集合；不默默忽略未识别内容。
def keys(obj, allowed, where):
    if not isinstance(obj, dict):
        raise InputError(f'{where} must be an object')
    if set(obj) - allowed:
        raise InputError(f'{where}: unknown fields {sorted(set(obj) - allowed)}')


# 只接受有限 int/float，排除 bool；显式允许的 null 编码为回放协议的 nan，表示未知而非零。
def number(value, where, nullable=False):
    if value is None and nullable:
        return 'nan'
    if type(value) not in (int, float) or not math.isfinite(value):
        raise InputError(f'{where} must be a finite number' + (' or null' if nullable else ''))
    return repr(value)


# 序号、时间和计数是有范围的无符号整数；bool 虽是 Python int 子类也不能通过。
def integer(value, where, maximum=2**64 - 1):
    if type(value) is not int or not 0 <= value <= maximum:
        raise InputError(f'{where} must be an unsigned integer <= {maximum}')
    return str(value)


# 必须是 JSON true/false；0、1 或非空字符串不冒充健康声明。
def boolean(value, where):
    if type(value) is not bool:
        raise InputError(f'{where} must be true or false')
    return '1' if value else '0'


# 回放驱动使用文本协议，故只接受可打印 ASCII，并用 JSON 转义保留单个字符串边界。
def string(value, where):
    if not isinstance(value, str) or any(ord(c) < 32 or ord(c) > 126 for c in value):
        raise InputError(f'{where} must be a printable ASCII string')
    return json.dumps(value)


# 长度必须精确对应协议；整个向量缺失时展开为等长未知值，不能自动补零。
def vector(value, length, where):
    if value is None:
        value = [None] * length
    if not isinstance(value, list) or len(value) != length:
        raise InputError(f'{where} must contain exactly {length} numbers/nulls')
    return [number(v, where, nullable=True) for v in value]


# 按 C++ 驱动约定顺序编码实际状态；四元数 4 项，两个协方差各 36 项，valid_fields 独立表达可信性。
def encode_state(obj):
    keys(obj, STATE_KEYS, 'state')
    out = [
        integer(obj.get('sequence', 0), 'state.sequence'),
        integer(obj.get('stamp_ns', 0), 'state.stamp_ns'),
        integer(obj.get('reset_counter', 0), 'state.reset_counter', 2**32 - 1),
        number(obj.get('received_age_seconds', 0), 'state.received_age_seconds'),
    ]
    out += [
        string(obj.get(k, ''), f'state.{k}')
        for k in ('frame_id', 'body_frame_id', 'source_id', 'angular_velocity_source')
    ]
    out += [
        boolean(obj.get(k, False), f'state.{k}')
        for k in ('initialized', 'tracking', 'calibration_valid')
    ]
    out += [integer(obj.get('valid_fields', 0), 'state.valid_fields', 65535)]
    for k, length in (
        ('position', 3),
        ('velocity', 3),
        ('attitude', 4),
        ('angular_velocity', 3),
        ('pose_covariance', 36),
        ('twist_covariance', 36),
    ):
        out += vector(obj.get(k), length, f'state.{k}')
    out += [number(obj.get('quality'), 'state.quality', nullable=True)]
    return out


# 目标模式必须显式属于白名单；位置、偏航和摇杆仍分别保留有效标志，数值存在不等于可用。
def encode_command(obj):
    keys(obj, COMMAND_KEYS, 'command')
    mode = obj.get('mode', 'Disabled')
    if not isinstance(mode, str) or mode not in MODES:
        raise InputError(f'command.mode must be one of {list(MODES)}')
    out = [
        integer(obj.get('sequence', 0), 'command.sequence'),
        integer(obj.get('stamp_ns', 0), 'command.stamp_ns'),
        integer(obj.get('reset_counter', 0), 'command.reset_counter', 2**32 - 1),
        number(obj.get('received_age_seconds', 0), 'command.received_age_seconds'),
        string(obj.get('source_id', ''), 'command.source_id'),
        string(obj.get('frame_id', ''), 'command.frame_id'),
        str(MODES[mode]),
    ]
    out += [
        boolean(obj.get(k, False), f'command.{k}')
        for k in ('valid', 'position_valid', 'yaw_valid', 'sticks_valid')
    ]
    out += vector(obj.get('position'), 3, 'command.position')
    out += [number(obj.get('yaw'), 'command.yaw', nullable=True)]
    out += vector(obj.get('sticks'), 4, 'command.sticks')
    return out


# 先读完整数据集，拒绝 JSON 的 NaN/Infinity 扩展与任何重复键；首行必须说明版本和 synthetic/recorded。
def load_jsonl(path):
    def constant(value):
        raise InputError(f'non-JSON constant {value}; use null for missing measurements')

    def unique(pairs):
        obj = {}
        for k, v in pairs:
            if k in obj:
                raise InputError(f'duplicate field {k}')
            obj[k] = v
        return obj

    records = []
    for n, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line, parse_constant=constant, object_pairs_hook=unique))
        except (ValueError, TypeError) as exc:
            raise InputError(f'{path}:{n}: {exc}') from exc
    if not records:
        raise InputError('empty JSONL session')
    session = records.pop(0)
    keys(
        session,
        {'schema_version', 'kind', 'dataset_kind', 'description', 'candidate_parameters'},
        'session',
    )
    if (
        session.get('schema_version') != 1
        or type(session.get('schema_version')) is not int
        or session.get('kind') != 'session'
    ):
        raise InputError('first record must be kind=session, schema_version=1')
    if session.get('dataset_kind') not in ('synthetic', 'recorded'):
        raise InputError('session.dataset_kind must be synthetic or recorded')
    if not records:
        raise InputError('session needs at least one tick')
    return session, records


# 所有 tick 先完整校验再交给驱动；dt 限制为 (0,1] 秒，PX4 反馈、状态和指令各有独立存在位。
def encode_records(session, records):
    candidates = session.get('candidate_parameters', {})
    keys(candidates, CANDIDATES, 'candidate_parameters')
    out = ['DATA ' + session['dataset_kind']]
    for name, value in sorted(candidates.items()):
        out.append(
            'V '
            + string(name, 'candidate name')
            + ' '
            + number(value, f'candidate_parameters.{name}')
        )
    for index, row in enumerate(records, 2):
        keys(
            row,
            {
                'kind',
                'dt',
                'ros_now_ns',
                'state',
                'command',
                'state_publishers',
                'command_publishers',
                'px4',
            },
            f'line {index}',
        )
        if row.get('kind') != 'tick':
            raise InputError(f'line {index}: kind must be tick')
        dt = row.get('dt', 0.05)
        number(dt, 'dt')
        if dt <= 0 or dt > 1:
            raise InputError('tick.dt must be > 0 and <= 1 second')
        px4 = row.get('px4')
        if px4 is not None:
            keys(px4, {'height', 'up_speed', 'yaw'}, 'px4')
        parts = [
            'T',
            number(dt, 'dt'),
            integer(row.get('ros_now_ns', 0), 'ros_now_ns'),
            integer(row.get('state_publishers', 1), 'state_publishers'),
            integer(row.get('command_publishers', 1), 'command_publishers'),
            '1' if px4 is not None else '0',
        ]
        if px4 is not None:
            parts += [
                number(px4.get('height', 0.5), 'px4.height'),
                number(px4.get('up_speed', 0), 'px4.up_speed'),
                number(px4.get('yaw', 0), 'px4.yaw'),
            ]
        parts += ['1' if row.get('state') is not None else '0']
        if row.get('state') is not None:
            parts += encode_state(row['state'])
        parts += ['1' if row.get('command') is not None else '0']
        if row.get('command') is not None:
            parts += encode_command(row['command'])
        out.append(' '.join(parts))
    return out


# 从当前 ROS 参数头提取实际成员映射，避免离线驱动另维护一份易漂移的参数表；数量异常立即要求人工核对。
def parameter_mapping():
    header = (INCLUDE / 'jikl_fc01_system/attitude_ros_parameters.hpp').read_text()
    direct = re.findall(
        r'c\.([\w.]+)\s*=\s*node\.declare_parameter<(bool|double|std::string|std::int64_t)>\("([^"]+)"',
        header,
    )
    doubles = re.findall(r'\{"([^"]+)",&c\.([\w.]+)\}', header)
    if len(direct) < 9 or len(doubles) < 45:
        raise RuntimeError('actual ROS parameter mapping changed; review replay mapping')
    lines = []
    for member, kind, name in direct:
        expr = (
            'strict_bool(value)'
            if kind == 'bool'
            else (
                'std::stod(value)'
                if kind == 'double'
                else 'std::stoll(value)' if kind == 'std::int64_t' else 'value'
            )
        )
        lines.append(f'if(key=="{name}"){{cfg.attitude.{member}={expr};return true;}}')
    for name, member in doubles:
        lines.append(f'if(key=="{name}"){{cfg.attitude.{member}=std::stod(value);return true;}}')
    return '\n'.join(lines) + '\n'


# 只编译宿主机离线驱动：源码与参数映射共同计算缓存哈希，新构建先入临时目录再原子替换。
def driver():
    compiler = shutil.which('g++')
    if compiler is None:
        raise RuntimeError('g++ is required for the no-hardware native replay')
    mapping = parameter_mapping()
    digest = hashlib.sha256(mapping.encode())
    # 依赖头文件和测试支持内容均进入摘要；任一变化都会选择新缓存，不能复用旧算法的回放结果。
    sources = sorted((INCLUDE / 'jikl_fc01_system').glob('*.hpp')) + [
        ROOT / 'native/tools/vision_replay.cpp',
        ROOT / 'native/tests/vision_test_support.hpp',
        ROOT / 'native/tests/authority_test_support.hpp',
    ]
    for path in sources:
        digest.update(path.read_bytes())
    cache = ROOT / '.superpowers/vision_replay' / digest.hexdigest()[:20]
    binary = cache / 'vision_replay'
    if not binary.is_file():
        cache.mkdir(parents=True, exist_ok=True)
        (cache / 'vision_replay_parameters.inc').write_text(mapping)
        with tempfile.TemporaryDirectory(prefix='compile-', dir=cache) as td:
            target = Path(td) / 'vision_replay'
            command = [
                compiler,
                '-std=c++17',
                '-O2',
                '-Wall',
                '-Wextra',
                '-pedantic',
                '-I',
                str(INCLUDE),
                '-I',
                str(ROOT / 'native/tests'),
                '-I',
                str(cache),
                str(ROOT / 'native/tools/vision_replay.cpp'),
                '-o',
                str(target),
            ]
            built = subprocess.run(command, capture_output=True, text=True, timeout=90)
            if built.returncode:
                raise RuntimeError('native replay compilation failed:\n' + built.stderr)
            os.replace(target, binary)
    return binary


# 读取当前 YAML 的参数映射；布尔、有限数字与字符串转为驱动协议，复杂非标量保持既有跳过行为。
def configuration(path):
    raw = yaml.safe_load(path.read_text())
    try:
        params = raw['fc01_system']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise InputError('config needs fc01_system.ros__parameters') from exc
    if not isinstance(params, dict):
        raise InputError('config parameters must be a mapping')
    lines = []
    for key, value in params.items():
        if type(value) is bool:
            value = 'true' if value else 'false'
        elif isinstance(value, (float, int)):
            value = number(value, f'config.{key}')
        elif not isinstance(value, str):
            continue
        lines.append('P ' + string(key, 'parameter') + ' ' + string(value, 'parameter value'))
    return lines


# 理想模型只积分最终提交的姿态/推力并回灌同一 Inbox/Runtime；不模拟完整真实机动力学。
def run_demo(binary, config, name):
    # Native driver integrates only final submitted q/thrust, then feeds the
    # ideal state back through the same Inbox and Runtime on every 20 Hz tick.
    text = '\n'.join(config + [f'DEMO {name}']) + '\n'
    return subprocess.run([str(binary)], input=text, text=True, timeout=30).returncode


# JSONL 与 --demo 必须二选一；先校验输入，之后才编译/启动本机驱动，异常返回 2。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', nargs='?', type=Path, help='version 1 JSONL replay session')
    parser.add_argument(
        '--demo', choices=('cruise', 'height'), help='ideal closed-loop synthetic scenario'
    )
    parser.add_argument(
        '--config', type=Path, default=ROOT / 'native/config/attitude_altitude.yaml'
    )
    args = parser.parse_args(argv)
    if bool(args.session) == bool(args.demo):
        parser.error('provide exactly one JSONL session or --demo')
    try:
        params = configuration(args.config)
        if args.demo:
            return run_demo(driver(), params, args.demo)
        session, records = load_jsonl(args.session)
        encoded = encode_records(
            session, records
        )  # Complete validation before compiling/starting native driver.
        return subprocess.run(
            [str(driver())], input='\n'.join(params + encoded) + '\n', text=True, timeout=30
        ).returncode
    except (InputError, OSError, RuntimeError, subprocess.TimeoutExpired, yaml.YAMLError) as exc:
        print(f'replay input: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
