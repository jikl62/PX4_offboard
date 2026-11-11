#!/usr/bin/env python3
# 历史 R6 本地审计工具：检查 YAML 动作门和捕获的双发布事件，不向 PX4 发消息。
# 报告明确区分本地 sink 提交与飞控采用；任何本地通过都不会把 powered_flight_go 设为 true。
"""Inspect local configuration/captured logs. No ROS, ports or command writes."""
import argparse
import json
import math
from pathlib import Path
import yaml

GATES = ('flight_actions_enabled', 'arm_enabled', 'takeoff_enabled', 'flight_parameters_confirmed')


# 检查四个动作门确实为 False，并单独记录 PX4 LAND 合同是否配置；外部执行仍保持未验证。
def audit_parameters(parameters):
    closed = all(parameters.get(k) is False for k in GATES)
    fallback = parameters.get('px4_land_fallback_confirmed') is True
    return {
        'closed_gate_check_pass': closed,
        'four_gate_values': {k: parameters.get(k) for k in GATES},
        'px4_land_contract_configured': fallback,
        'external_fallback_execution_verified': False,
        'powered_flight_go': False,
        'scope': 'LOCAL_CONFIG_CHECK_ONLY',
        'remaining': [
            'C_R6_INDEPENDENT_REVIEW',
            'NATIVE_HUMBLE_BUILD',
            'ACTUAL_FC01_FALLBACK_WITH_CURRENT_SENSORS',
            'ACTUAL_AIRFRAME_CALIBRATION',
        ],
    }


# 从 ROS 参数节点读取 YAML 映射；结构错误由上层异常路径拒绝。
def check_config(path):
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    parameters = document['fc01_system']['ros__parameters']
    if not isinstance(parameters, dict):
        raise ValueError('invalid ROS parameters')
    return audit_parameters(parameters)


# 按单调时间扫描 phase_event：统计目标/心跳部分发布、异常、已解锁 Offboard 缺失和最大间隔。
def check_events(rows):
    events = [r for r in rows if r.get('kind') == 'phase_event']
    partial = 0
    missing = 0
    exceptions = 0
    last_tick = None
    last_mode = None
    max_gap = 0.0
    ordered = True
    commits = 0
    for row in events:
        stamp = row.get('monotonic_s')
        # bool 虽是 Python int 子类，但不是合法时间；非有限或乱序时间会使顺序证据失效。
        if (
            isinstance(stamp, bool)
            or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp)
        ):
            ordered = False
            continue
        if last_tick is not None and stamp <= last_tick:
            ordered = False
        last_tick = stamp
        target = row.get('target_local_commit') is True
        mode = row.get('heartbeat_local_commit') is True
        if row.get('sink_exception') is True:
            exceptions += 1
        # 同一周期只有一个 sink 提交属于部分发布，不得让后续健康帧抹掉计数。
        if target != mode:
            partial += 1
        if row.get('armed') is True and row.get('nav_state') == 14 and (not target or not mode):
            missing += 1
        if mode:
            commits += 1
            if last_mode is not None:
                max_gap = max(max_gap, stamp - last_mode)
            last_mode = stamp
        gap = row.get('max_heartbeat_gap_s', 0)
        if isinstance(gap, (int, float)) and math.isfinite(gap):
            max_gap = max(max_gap, gap)
    # 末尾到最后心跳的间隙也必须计入，防止日志尾部停止发布被正常周期均值掩盖。
    if last_tick is not None and last_mode is not None:
        max_gap = max(max_gap, last_tick - last_mode)
    return {
        'phase_event_rows': len(events),
        'heartbeat_local_commits': commits,
        'partial_sink_ticks': partial,
        'max_local_heartbeat_gap_s': max_gap,
        'monotonic_order_valid': ordered,
        'armed_offboard_missing_sink_ticks': missing,
        'sink_exception_ticks': exceptions,
        'local_dual_sink_check_pass': bool(events)
        and commits > 0
        and ordered
        and partial == 0
        and missing == 0
        and exceptions == 0
        and max_gap <= 0.3,
        'px4_receipt_proven': False,
        'fallback_execution_verified': False,
    }


# 配置是必需输入；可选事件记录必须同时通过双 sink 检查才返回成功。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--events', type=Path)
    args = parser.parse_args()
    try:
        result = check_config(args.config)
        if args.events:
            rows = [
                json.loads(line)
                for line in args.events.read_text(encoding='utf-8').splitlines()
                if line.strip().startswith('{')
            ]
            result['captured_event_check'] = check_events(rows)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        ok = result['closed_gate_check_pass']
        if args.events:
            ok = ok and result['captured_event_check']['local_dual_sink_check_pass']
        return 0 if ok else 1
    except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        print(
            json.dumps(
                {'closed_gate_check_pass': False, 'powered_flight_go': False, 'error': str(error)}
            )
        )
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
