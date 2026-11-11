#!/usr/bin/env python3
# 正式RC导航服务入口：生成诊断或人工解锁起飞配置，校验当前二进制标识，再交给standby监督。
# 目标高度相对地面向上为正，默认1.5m；视觉始终关闭，Python不产生姿态/推力设定值。
"""Native manual-RC navigation service; Python never emits flight setpoints."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import yaml

from fc01_control import ControlError, DEFAULT_TAKEOFF_HEIGHT_M, runtime_profile
from launcher import PROJECT, private_dir, verify_binary
from navigation_status import controller_state_current, preparation_diagnostic_reason
import standby

ENTRY = Path(__file__).resolve().parent
# 二进制内标记是版本能力检查，仍需构建摘要与依赖证据确认实际来源。
MARKER = b'RC_NAVIGATION_MANUAL_ARM_V1'
TAKEOFF_MARKER = b'NAV_TAKEOFF_EVIDENCE_V2_20261103'
SAFETY_MARKER = b'R2_DISCRETE_EVENT_PREFLIGHT_V2_20261108'
LIFECYCLE_MARKER = b'FC01_RECOVERY_DIAGNOSTIC_V5_20261110'
ENTRY_VERSION = '20261110.7'
DIAGNOSTIC = 'diagnostic'
FLIGHT_TEST = 'flight-test'
MODES = (DIAGNOSTIC, FLIGHT_TEST)


# 模式白名单决定源YAML；默认诊断不共享旧运行YAML以免继承飞行门。
def candidate_config_path(mode, entry=ENTRY):
    if mode not in MODES:
        raise ControlError('运行模式必须为 diagnostic 或 flight-test')
    return (
        Path(entry)
        / 'native'
        / 'config'
        / ('diagnostic.yaml' if mode == DIAGNOSTIC else 'flight_test.yaml')
    )


# 只写身份路径/版本，供报告对齐真正运行源，不构成动作许可。
def _metadata(config, mode, *, config_path=None, source_path=None):
    p = config['fc01_system']['ros__parameters']
    p['fc01_runtime_mode'] = mode
    p['fc01_entry_version'] = ENTRY_VERSION
    if config_path is not None:
        p['fc01_config_path'] = str(Path(config_path).expanduser().resolve())
    if source_path is not None:
        p['fc01_config_source_path'] = str(Path(source_path).expanduser().resolve())
    return config


# 深拷贝原候选后明确关闭全部动作/视觉门、禁用CSV，目标仅用于诊断显示。
def diagnostic_profile(base, target=DEFAULT_TAKEOFF_HEIGHT_M):
    config = runtime_profile(base, False)
    p = config['fc01_system']['ros__parameters']
    if (
        type(target) not in (int, float)
        or not math.isfinite(target)
        or not 0 < target < p['aa_max_height']
    ):
        raise ControlError('诊断目标高度必须为有限数字且位于 (0, aa_max_height)')
    p.update(
        rc_navigation_enabled=False,
        rc_target_height=float(target),
        rc_arm_to_navigation_timeout=8.0,
        flight_enabled=False,
        flight_actions_enabled=False,
        arm_enabled=False,
        takeoff_enabled=False,
        flight_parameters_confirmed=False,
        rc_takeover_confirmed=False,
        autonomous_failsafe_confirmed=False,
        bench_arm_only_enabled=False,
        visual_navigation_enabled=False,
        csv_path='',
    )
    return config


# 保留原10度/30秒包络；RC人工解锁路线只开动作总门与起飞门，程序arm与视觉均关闭。
def navigation_profile(base, target=DEFAULT_TAKEOFF_HEIGHT_M, mode_slot=6, mode=FLIGHT_TEST):
    if mode == DIAGNOSTIC:
        return diagnostic_profile(base, target)
    if mode != FLIGHT_TEST:
        raise ControlError('运行模式必须为 diagnostic 或 flight-test')
    config = runtime_profile(base, False)
    p = config['fc01_system']['ros__parameters']
    if (
        type(target) not in (int, float)
        or not math.isfinite(target)
        or not 0 < target < p['aa_max_height']
    ):
        raise ControlError('导航高度必须为有限数字且位于 (0, aa_max_height)')
    if p['aa_max_tilt_deg'] > 10 or p['aa_max_flight_seconds'] > 30:
        raise ControlError('导航沿用最大 10 度/30 秒限制')
    if type(mode_slot) is not int or not 1 <= mode_slot <= 6:
        raise ControlError('导航 mode_slot 必须是 1–6 的整数；对应飞控模式档位，不是 CH6 通道号')
    p.update(
        rc_navigation_enabled=True,
        rc_target_height=float(target),
        rc_navigation_mode_slot=mode_slot,
        rc_arm_to_navigation_timeout=8.0,
        flight_enabled=False,
        flight_actions_enabled=True,
        arm_enabled=False,
        takeoff_enabled=True,
        flight_parameters_confirmed=True,
        rc_takeover_confirmed=True,
        autonomous_failsafe_confirmed=False,
        bench_arm_only_enabled=False,
        visual_navigation_enabled=False,
        csv_path='',
    )
    return config


# 读取导航高度/档位选择；缺少设置只用原默认高度，最终数值仍由profile验证。
def settings(entry=ENTRY):
    path = entry / 'navigation_settings.json'
    return (
        json.loads(path.read_text())
        if path.exists()
        else {'target_height_m': DEFAULT_TAKEOFF_HEIGHT_M}
    )


# 根据显式prefix或当前entry/install定位唯一原生节点，不从PATH猜控制器。
def controller_path(entry=ENTRY, prefix=None):
    return (prefix or entry / 'install') / 'lib/jikl_fc01_system/fc01_system_node'


# 先验真实可执行架构，再检查人工解锁、起飞证据、离散事件和生命周期四个能力标识。
def check_controller(entry=ENTRY, prefix=None):
    binary = controller_path(entry, prefix)
    verify_binary(binary)
    if MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少手动解锁导航标识，保留原服务')
    if TAKEOFF_MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少本版起飞证据标识，请重新构建；保留原服务')
    if SAFETY_MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少本版离散安全事件标识，请重新构建；保留原服务')
    if LIFECYCLE_MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少本版安全生命周期标识，请全新构建；保留原服务')
    return binary


# 验证源YAML/环境并合成当前模式；返回含真实配置来源路径的新配置对象。
def load_profile(root, agent, entry=ENTRY, mode=FLIGHT_TEST, *, config_path=None):
    source = candidate_config_path(mode, entry)
    base = standby.validate(root, agent, config_path=source)
    selected = settings(entry)
    slot = selected.get(
        'navigation_mode_slot',
        base['fc01_system']['ros__parameters'].get('rc_navigation_mode_slot', 6),
    )
    config = navigation_profile(
        base, selected.get('target_height_m', DEFAULT_TAKEOFF_HEIGHT_M), slot, mode
    )
    # Visual navigation is deliberately disabled in both operator modes.
    return _metadata(config, mode, config_path=config_path, source_path=source)


# 展示层把当前原生phase映射友好状态；历史控制器数据不能用作本次故障锁存或准备提示。
def describe_row(row):
    state = row.get('state') or {}
    phase = state.get('phase')
    result = dict(row)
    native_current = controller_state_current(row)
    result['controller_state_current'] = native_current
    result['fault_latched'] = native_current and phase == 'SAFETY'
    reason = state.get('reason') or row.get('reason', '')
    if result['fault_latched']:
        result.update(status='FAULT', reason=reason)
        return result
    if not native_current or row.get('feedback_current') is not True or not phase:
        return result
    if row.get('status') == 'CONNECTED':
        diagnostic = preparation_diagnostic_reason(state)
        if diagnostic:
            result.update(status='DIAGNOSTIC', reason=diagnostic)
            return result
        names = {
            'RC_PRESTREAM': 'PRESTREAM',
            'RC_READY_WAIT_ARM': 'READY_WAIT_ARM',
            'RC_ARMED_WAIT_NAVIGATION': 'WAIT_NAVIGATION',
            'ARMED_GROUND': 'ARMED_GROUND',
            'THRUST_RAMP': 'THRUST_RAMP',
            'ASCEND': 'ASCEND',
            'HEIGHT_HOLD': 'HEIGHT_HOLD',
            'DESCENT': 'LANDING',
            'RECOVERY_DESCENT': 'LANDING',
            'POWERED_GROUND_UNCONFIRMED': 'LANDING',
            'LANDED': 'LANDING',
            'REQUEST_DISARM': 'WAIT_DISARM',
            'SAFE_DISARMED': 'SAFE_DISARMED',
            'RELEASE_OFFBOARD': 'RELEASING',
        }
        result.update(status=names.get(phase, 'WAITING'), reason=reason)
    else:
        result.update(status='WAITING', reason=reason)
    return result


def _parse_service_args(argv):
    """解析并校验入口参数，返回(args, parser)；仅Flight Test双重确认成立才选择该模式。

    本函数不启动服务、不改配置，错误顺序保留。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'action',
        choices=('run', 'prepare-config', 'validate-env', 'check-config', 'probe', 'build'),
        nargs='?',
        default='run',
    )
    parser.add_argument(
        '--mode',
        choices=MODES,
        default=DIAGNOSTIC,
        help='diagnostic（默认只读）或 flight-test（必须显式确认）',
    )
    parser.add_argument('--flight-test', action='store_true', help='--mode flight-test 的显式别名')
    parser.add_argument(
        '--confirm-flight-test', action='store_true', help='确认本次人工 Flight Test 选择'
    )
    parser.add_argument('--file', type=Path)
    parser.add_argument('--prefix', type=Path)
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument(
        '--require-existing-agent',
        action='store_true',
        help='部署探针仅复用既有 Agent；不创建新的 Agent',
    )
    parser.add_argument(
        '--require-rc-safe', action='store_true', help='部署探针另需新鲜、安全且单一来源的 RC 反馈'
    )
    args = parser.parse_args(argv)
    if args.flight_test:
        args.mode = FLIGHT_TEST
    if args.mode == FLIGHT_TEST and not args.confirm_flight_test:
        parser.error('Flight Test 必须同时提供 --confirm-flight-test；不会隐式开启动作门')
    return args, parser


def _check_runtime_config(args, parser):
    """检查所选配置并返回退出码；diagnostic逐门确切False，flight-test调用既有配置检查器。

    按原错误优先级执行，未提供有效配置时不启动控制器。
    """
    if args.file is None:
        parser.error('check-config requires --file')
    try:
        config = yaml.safe_load(args.file.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ControlError('读取配置失败：' + str(exc)) from exc
    if (
        not isinstance(config, dict)
        or not isinstance(config.get('fc01_system'), dict)
        or not isinstance(config['fc01_system'].get('ros__parameters'), dict)
    ):
        raise ControlError('配置缺少 fc01_system/ros__parameters')
    p = config['fc01_system']['ros__parameters']
    # 诊断check-config只接受所有门确切False；飞行候选则调用已构建C++同一Config校验。
    if args.mode == DIAGNOSTIC:
        required = (
            'flight_enabled',
            'flight_actions_enabled',
            'arm_enabled',
            'takeoff_enabled',
            'rc_navigation_enabled',
            'bench_arm_only_enabled',
            'visual_navigation_enabled',
            'flight_parameters_confirmed',
            'rc_takeover_confirmed',
            'autonomous_failsafe_confirmed',
        )
        if any(p.get(key) is not False for key in required):
            raise ControlError('diagnostic 配置动作门/视觉必须全部关闭')
        return 0
    check_controller(prefix=args.prefix)
    checker = (args.prefix or ENTRY / 'install') / 'lib/jikl_fc01_system/fc01_config_check'
    if not checker.is_file() or not os.access(checker, os.X_OK):
        raise ControlError('缺少已编译的原生配置检查器')
    return subprocess.run(
        [
            str(checker),
            '--ros-args',
            '--disable-external-lib-logs',
            '--params-file',
            str(args.file),
        ],
        check=False,
    ).returncode


def _run_navigation_service(args, root, agent):
    """按已确认模式构造运行配置并交给既有standby监督器；信号只置停止事件。保留打印、控制器核验与配置工厂调用顺序，不新增解锁或飞行权限。"""
    # 信号只置停止事件；standby负责在Agent还活着时等待原生恢复/释放，不能直接杀电机控制进程。
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    runtime_path = Path.home() / 'FC01_navigation' / 'runtime.yaml'
    profile = load_profile(root, agent, ENTRY, args.mode, config_path=runtime_path)
    p = profile['fc01_system']['ros__parameters']
    print(
        'FC01 导航启动 version=%s mode=%s hover_thrust=%.3f reserve=%.3f target=%.3f permissions=%s visual=%s config=%s'
        % (
            ENTRY_VERSION,
            args.mode,
            p.get('aa_hover_thrust', float('nan')),
            p.get('aa_takeoff_reserve_thrust', 0.0),
            p.get('rc_target_height', DEFAULT_TAKEOFF_HEIGHT_M),
            ','.join(
                '%s=%s' % (k, p.get(k))
                for k in (
                    'flight_actions_enabled',
                    'arm_enabled',
                    'takeoff_enabled',
                    'rc_navigation_enabled',
                )
            ),
            p.get('visual_navigation_enabled', False),
            runtime_path,
        ),
        flush=True,
    )
    standby.supervise(
        root,
        agent,
        Path.home() / 'FC01_navigation',
        stopped,
        controller=controller_path(),
        config_factory=lambda r, a: (
            check_controller(),
            load_profile(r, a, ENTRY, args.mode, config_path=runtime_path),
        )[1],
        status_metadata={
            'runtime_mode': args.mode,
            'entry_version': ENTRY_VERSION,
            'config_path': str(runtime_path),
            'config_source_path': str(candidate_config_path(args.mode)),
            'rc_navigation_enabled': p.get('rc_navigation_enabled', False),
            'flight_enabled': p.get('flight_enabled', False),
            'flight_actions_enabled': p.get('flight_actions_enabled', False),
            'arm_enabled': p.get('arm_enabled', False),
            'takeoff_enabled': p.get('takeoff_enabled', False),
            'rc_target_height': p.get('rc_target_height', DEFAULT_TAKEOFF_HEIGHT_M),
            'aa_hover_thrust': p.get('aa_hover_thrust'),
            'aa_takeoff_reserve_thrust': p.get('aa_takeoff_reserve_thrust'),
            'automatic_arm': False,
            'visual_navigation_enabled': False,
        },
        config_path=candidate_config_path(args.mode),
        row_formatter=describe_row,
    )


# 显式子命令分离环境/构建/配置/探针与run；默认诊断，flight-test没有确认直接拒绝。
def main(argv=None):
    args, parser = _parse_service_args(argv)
    os.umask(0o077)
    root = (
        Path(os.environ.get('FC01_PROJECT_ROOT', str(Path.home() / PROJECT))).expanduser().resolve()
    )
    agent = Path.home() / 'fc01_agent_build_offline/MicroXRCEAgent'
    if args.action == 'validate-env':
        load_profile(root, agent, ENTRY, args.mode)
        print('ROS / 消息 / 原生工程 / %s 配置检查通过。' % args.mode)
    elif args.action == 'prepare-config':
        if args.file is None:
            parser.error('prepare-config requires --file')
        private_dir(args.file.parent)
        args.file.write_text(
            yaml.safe_dump(
                load_profile(root, agent, ENTRY, args.mode, config_path=args.file),
                allow_unicode=True,
                sort_keys=False,
            )
        )
    elif args.action == 'build':
        if args.prefix is None or args.build_dir is None:
            parser.error('build requires --prefix and --build-dir')
        from navigation_install import build_native

        build_native(ENTRY, args.prefix, args.build_dir)
    elif args.action == 'check-config':
        return _check_runtime_config(args, parser)
    elif args.action == 'probe':
        profile = load_profile(root, agent, ENTRY, args.mode)
        from navigation_probe import probe

        if args.require_rc_safe:
            return probe(
                root,
                agent,
                args.file,
                require_existing_agent=args.require_existing_agent,
                require_rc_safe=True,
                rc_navigation_mode_slot=profile['fc01_system']['ros__parameters'][
                    'rc_navigation_mode_slot'
                ],
            )
        if args.require_existing_agent:
            return probe(root, agent, args.file, require_existing_agent=True)
        return probe(root, agent, args.file)
    else:
        _run_navigation_service(args, root, agent)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
