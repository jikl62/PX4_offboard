#!/usr/bin/env python3
# 原生 Unix socket 现场控制台：连接本身不解锁，动作请求由原生端再次验证。
# 命令行单次请求限制为检查入口；飞行/移动操作要求现场交互，台架 ARM_ONLY 还有独立确认流程。
"""R6.2c FC01 operator console. No ROS dependency; never arms on connection."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import shlex
import socket
import sys
import time


# 只接受白名单和精确参数数量；速度单位 m/s、偏航角速度 rad/s，单次定时移动时长最多 2 秒。
def parse_command(text):
    parts = shlex.split(text)
    if not parts:
        raise ValueError('请输入命令')
    op = parts[0].lower()
    if op == 'authority':
        routes = {
            'status': 'AUTHORITY_STATUS',
            'rc': 'AUTHORITY_RC',
            'autonomous': 'AUTHORITY_AUTONOMOUS',
            'reset': 'AUTHORITY_RESET',
            'takeover': 'AUTHORITY_TAKEOVER',
        }
        if len(parts) != 2 or parts[1].lower() not in routes:
            raise ValueError('用法: authority status|rc|autonomous|reset|takeover')
        return routes[parts[1].lower()], [], 0.0
    names = {
        'status': 'STATUS',
        'mode': 'MODE',
        'attitude-ready': 'ATTITUDE_READY',
        'takeoff': 'TAKEOFF',
        'arm-only': 'ARM_ONLY',
        'vel': 'VEL',
        'hold': 'HOLD',
        'land': 'LAND',
        'update-check': 'UPDATE_CHECK',
        'stop': 'STOP',
        'quit': 'QUIT',
        'help': 'HELP',
    }
    if op not in names:
        raise ValueError('未知命令；输入 help')
    expected = {'takeoff': 1, 'vel': 5}.get(op, 0)
    if len(parts) != expected + 1:
        raise ValueError('参数数量不正确')
    values = [float(x) for x in parts[1:]]
    if not all(math.isfinite(x) for x in values):
        raise ValueError('参数必须是有限数字')
    if op == 'takeoff' and values[0] <= 0:
        raise ValueError('高度必须大于 0')
    duration = 0.0
    if op == 'vel':
        duration = values.pop()
        if not 0 < duration <= 2:
            raise ValueError('单次移动持续时间必须在 (0, 2] 秒内')
    return names[op], values, duration


# 优先读取 FC01_SOCKET，否则连接当前用户缓存目录；不会扫描或删除其他会话的 socket。
def socket_path():
    return os.environ.get('FC01_SOCKET', str(Path.home() / '.cache/fc01/control.sock'))


# 只格式化已经收到的结果；原始 JSON 模式保留全部字段，地面失败位掩码展开为便于操作者理解的原因。
def show(obj, raw):
    if raw:
        print(json.dumps(obj, ensure_ascii=False), flush=True)
    elif obj.get('kind') == 'state' or (
        obj.get('kind') == 'reply'
        and obj.get('message') in {'STATUS', 'AUTHORITY_STATUS'}
        and 'phase' in obj
    ):
        print(
            '[{phase}] mode={nav_state} armed={armed} valid={estimate_valid} ground={ground_test} reason={reason}'.format(
                **obj
            ),
            flush=True,
        )
        if 'authority_mode' in obj:
            print(
                'authority={} locked={} ready={} reason={} takeover={}'.format(
                    obj['authority_mode'],
                    obj.get('authority_locked'),
                    obj.get('authority_ready'),
                    obj.get('authority_reason'),
                    obj.get('takeover_reason'),
                ),
                flush=True,
            )
        if obj.get('bench_operator_required') is True:
            print('台架等待超时：需要操作者确认或接管；程序仍禁止盲目取消解锁。', flush=True)
    else:
        print(
            '[{}] {}'.format('OK' if obj.get('accepted') else 'BLOCKED', obj.get('message', obj)),
            flush=True,
        )
        if (
            obj.get('message') == 'AUTHORITY_SAFE_GROUND_REQUIRED'
            and obj.get('last_authority_ground_evidence_valid') is True
        ):
            mask = obj.get('last_authority_ground_failed_mask', 0)
            labels = [
                label
                for bit, label in (
                    (1, '仍已解锁'),
                    (2, '未确认接地'),
                    (4, '未静止'),
                    (8, '连续静止时间不足'),
                    (16, 'Vz超限'),
                )
                if mask & bit
            ]
            print(
                '地面拒绝样本：{}；静止 {} / {} s，Vz={} m/s，允许 |Vz|<={} m/s'.format(
                    '、'.join(labels),
                    obj.get('last_authority_ground_stationary_seconds'),
                    obj.get('required_ground_seconds'),
                    obj.get('last_authority_ground_vz_ned'),
                    obj.get('ground_vz_tolerance'),
                ),
                flush=True,
            )


def _initial_console_command(args):
    """解析一次性请求并检查TTY权限，返回初始命令或None；飞行动作/权限切换只允许现场控制台。

    先完成原拒绝检查再建立socket，错误文本与顺序不变。
    """
    initial = parse_command(args.command) if args.command else None
    # 单次 --command 不能提交飞行、运动或权限切换；避免管道脚本在没有现场交互的情况下发动作。
    if initial and initial[0] in {
        'TAKEOFF',
        'ARM_ONLY',
        'VEL',
        'HOLD',
        'LAND',
        'AUTHORITY_RC',
        'AUTHORITY_AUTONOMOUS',
        'AUTHORITY_RESET',
        'AUTHORITY_TAKEOVER',
    }:
        raise ValueError('Flight/motion commands require the live interactive console')
    if initial and initial[0] in {'HELP', 'QUIT'}:
        raise ValueError('Use the interactive console for help/quit')
    if not initial and not sys.stdin.isatty():
        raise ValueError(
            'Live interactive console requires a TTY; use --command for status/mode/attitude-ready'
        )
    return initial


def _arm_only_preflight(args, arm_attempt_consumed, latest_state, latest_state_at, now):
    """只读检查单次台架确认的最新状态并返回拒绝文本或None；now/state_at为单调秒，NED地面Vz单位m/s。

    调用者仍拥有唯一提交水位；本检查不发送ARM，原字段类型和首拒绝顺序严格保留。
    """
    # Secondary UI guard, not a substitute for the C++ runtime and PX4 checks.
    if not args.bench_arm_only:
        return '需要使用 --bench-arm-only 启动交互控制台'
    if arm_attempt_consumed:
        return '本控制台会话已经提交过 ARM_ONLY；禁止重复提交'
    if latest_state is None or now - latest_state_at > 1.0:
        return '没有最近1秒内的实时控制器状态'
    s = latest_state
    # 逐项核对专用台架状态；通用起飞门必须关闭，ARM_ONLY 允许也不表示 TAKEOFF 允许。
    expected = {
        'profile': 'attitude_altitude',
        'bench_arm_only_enabled': True,
        'phase': 'OFFBOARD_READY',
        'nav_state': 14,
        'armed': False,
        'landed': True,
        'at_rest': True,
        'status_fresh': True,
        'local_fresh': True,
        'land_fresh': True,
        'failsafe_fresh': True,
        'battery_fresh': True,
        'source_clock_valid': True,
        'source_clock_validated': True,
        'attitude_fresh': True,
        'z_valid': True,
        'vz_valid': True,
        'preflight': True,
        'battery_connected': True,
        'authority_mode': 'RC_TAKEOVER',
        'authority_ready': True,
        'authority_attempt_mode': 'RC_TAKEOVER',
        'authority_locked': True,
        'rc_link_ready': True,
        'rc_switches_ready': True,
        'rc_kill_latched': False,
        'flight_actions_enabled': True,
        'arm_enabled': True,
        'takeoff_enabled': False,
        'flight_parameters_confirmed': True,
        'publication_fault_latched': False,
        'authority_ground_failed_mask': 0,
    }
    for key, value in expected.items():
        if type(s.get(key)) is not type(value) or s.get(key) != value:
            return f'状态条件不满足：{key} 应为 {value!r}，实际 {s.get(key)!r}'
    for key in (
        'stationary_seconds',
        'required_ground_seconds',
        'ground_vz_ned',
        'ground_vz_tolerance',
    ):
        value = s.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            return f'缺少有效地面诊断数据：{key}'
    if s['required_ground_seconds'] < 2 or s['stationary_seconds'] < s['required_ground_seconds']:
        return '连续静止时间尚未满足控制器阈值'
    if s['ground_vz_tolerance'] <= 0 or abs(s['ground_vz_ned']) > s['ground_vz_tolerance']:
        return 'Vz尚未满足控制器地面阈值'
    return None


def _console_state_signature(obj):
    """返回界面节流所需状态签名；仅读取既有字段，不更新状态寿命、不影响ACK归属或命令提交。字段排列与原输出变化判定一致。"""
    return tuple(
        obj.get(k)
        for k in (
            'phase',
            'nav_state',
            'armed',
            'reason',
            'authority_mode',
            'authority_ready',
            'authority_ground_failed_mask',
            'source_clock_valid',
            'rc_kill_latched',
            'bench_operator_required',
        )
    )


def _print_console_help():
    """显示原两段帮助；无参数、返回None、无飞行或权限副作用。只由显式HELP输入调用，保留文本与输出顺序。"""
    print(
        '速度单位 m/s；左旋 rad/s；时长 <=2 s。mode / attitude-ready 需要 ACK+新模式状态；attitude-ready 不解锁。quit/断线清除旧速度并请求降落。',
        flush=True,
    )
    print(
        'authority rc/autonomous 只选择策略，不确认失效保护、不改 PX4 参数；准备后锁定。takeover 释放本次控制；reset 仅在安全地面、输出停止且事务结束后回到 UNSET。',
        flush=True,
    )


# 一次连接拥有独立递增序号、待回复表和最近状态；INTERACTIVE 后每 0.2 秒维持心跳，不能靠缓存续租。
def run(args):
    initial = _initial_console_command(args)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect(args.socket)
    sock.setblocking(False)
    seq = 0
    pending = {}
    buf = b''
    velocity = None
    velocity_until = 0.0
    last_ping = last_vel = 0.0
    target_seq = None
    latest_state = None
    latest_state_at = -1.0
    arm_confirmation_deadline = 0.0
    arm_attempt_consumed = False

    # 写前消耗序号，0.3 秒总期限内处理部分写；非 PING 请求进入待回复表，按回复序号关联。
    def send(op, values=()):
        nonlocal seq
        seq += 1
        packet = (
            str(seq) + ' ' + op + ''.join(' ' + format(x, '.17g') for x in values) + '\n'
        ).encode()
        # Requests are short, but use select/send loops so backpressure cannot lose a command.
        end = time.monotonic() + 0.3
        while packet:
            if time.monotonic() > end:
                raise TimeoutError('命令连接拥塞')
            _, w, _ = select.select([], [sock], [], 0.05)
            if w:
                sent = sock.send(packet)
                if not sent:
                    raise ConnectionError('命令连接关闭')
                packet = packet[sent:]
        if op != 'PING':
            pending[seq] = op
        return seq

    with sock:
        if initial:
            target_seq = send(initial[0], initial[1])
        else:
            print(
                '命令: status | authority status/rc/autonomous/reset/takeover | mode | attitude-ready | arm-only (专用台架) | takeoff 高度 | vel 前 左 上 左旋 时长 | hold | land | quit',
                flush=True,
            )
            send('INTERACTIVE')
            send('STATUS')
        deadline = time.monotonic() + args.timeout if initial else math.inf
        mode_wait = False
        ready_wait = False
        last_shown_signature = None
        last_shown_at = -1.0
        # 主循环同时读 socket 与现场输入；定时速度到期发 HOLD，等待期间持续收状态、维持心跳。
        while time.monotonic() < deadline:
            now = time.monotonic()
            if arm_confirmation_deadline and now > arm_confirmation_deadline:
                arm_confirmation_deadline = 0.0
                print('ARM_ONLY 二次确认超时，已取消；没有发送解锁命令', flush=True)
            if not initial and now - last_ping >= 0.2:
                send('PING')
                last_ping = now
            if velocity is not None:
                if now >= velocity_until:
                    send('HOLD')
                    velocity = None
                elif now - last_vel >= 0.1:
                    send('VEL', velocity)
                    last_vel = now
            readers = [sock]
            if not initial:
                readers.append(sys.stdin)
            readable, _, _ = select.select(readers, [], [], 0.05)
            # 接收字节先拼缓存再按换行拆 JSON；状态更新接收时刻，回复只能结束对应请求的等待。
            if sock in readable:
                chunk = sock.recv(8192)
                if not chunk:
                    raise ConnectionError('控制节点已断开；检查机载日志')
                buf += chunk
                if len(buf) > 131072:
                    raise ValueError('返回数据过长')
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    obj = json.loads(line)
                    if obj.get('kind') == 'error':
                        show(obj, args.json)
                        return 2
                    op = pending.pop(obj.get('seq'), None)
                    if obj.get('kind') == 'state':
                        latest_state = obj
                        latest_state_at = time.monotonic()
                        signature = _console_state_signature(obj)
                        should_show = (
                            args.json
                            or initial
                            or signature != last_shown_signature
                            or latest_state_at - last_shown_at >= 5
                        )
                        if (not initial or mode_wait or ready_wait) and should_show:
                            show(obj, args.json)
                            last_shown_signature = signature
                            last_shown_at = latest_state_at
                        if mode_wait and obj.get('ground_test') not in {
                            'WAITING',
                            'SENT',
                            'NOT_RUN',
                        }:
                            return 0 if obj.get('ground_test') == 'PASS' else 2
                        if ready_wait and obj.get('phase') in {
                            'OFFBOARD_READY',
                            'SAFETY',
                            'IDLE',
                            'SAFE_DISARMED',
                        }:
                            return (
                                0
                                if obj.get('phase') == 'OFFBOARD_READY'
                                and obj.get('armed') is False
                                else 2
                            )
                    elif op:
                        show(obj, args.json)
                        if initial and obj.get('seq') == target_seq:
                            if not obj.get('accepted'):
                                return 2
                            if initial[0] == 'MODE':
                                mode_wait = True
                            elif initial[0] == 'ATTITUDE_READY':
                                ready_wait = True
                            else:
                                return 0
            if not initial and sys.stdin in readable:
                text = sys.stdin.readline()
                if not text:
                    return 0
                text = text.strip()
                # 二次确认前重新检查全部实时状态与 10 秒窗口；提交后即消耗本会话唯一 ARM_ONLY 机会。
                if text == 'BENCH ARM CONFIRM' and arm_confirmation_deadline:
                    why = _arm_only_preflight(
                        args, arm_attempt_consumed, latest_state, latest_state_at, time.monotonic()
                    )
                    if why or time.monotonic() > arm_confirmation_deadline:
                        print('ARM_ONLY 取消：' + (why or '确认超时'), flush=True)
                        arm_confirmation_deadline = 0.0
                        continue
                    arm_confirmation_deadline = 0.0
                    arm_attempt_consumed = True
                    send('ARM_ONLY')
                    print(
                        'ARM_ONLY 已提交一次；请观察 PX4 Armed/Disarmed 实际状态，不得重复发送',
                        flush=True,
                    )
                    continue
                if arm_confirmation_deadline:
                    arm_confirmation_deadline = 0.0
                    print('已取消 ARM_ONLY 二次确认；未发送解锁命令', flush=True)
                try:
                    op, values, duration = parse_command(text)
                    if op == 'QUIT':
                        return 0
                    if args.bench_arm_only and op == 'TAKEOFF':
                        print('台架控制台禁止 TAKEOFF', flush=True)
                        continue
                    # 第一次 arm-only 只打开确认窗口，不发解锁；其他输入或超时取消，专用台架会话始终禁止 TAKEOFF。
                    if op == 'ARM_ONLY':
                        why = _arm_only_preflight(
                            args,
                            arm_attempt_consumed,
                            latest_state,
                            latest_state_at,
                            time.monotonic(),
                        )
                        if why:
                            print('ARM_ONLY 拒绝：' + why, flush=True)
                            continue
                        arm_confirmation_deadline = time.monotonic() + 10.0
                        print(
                            '注意：真实 FC01 解锁后电机可能自行转动。若已获独立台架安全放行，请在10秒内完整输入 BENCH ARM CONFIRM；否则任何输入均取消。',
                            flush=True,
                        )
                        continue
                    if op == 'HELP':
                        _print_console_help()
                        continue
                    # 任何新的操作先取消旧定时速度，防止旧运动在新指令后继续周期重发。
                    # Any new operator command cancels the previous timed movement.
                    if velocity is not None:
                        send('HOLD')
                        velocity = None
                    send(op, values)
                    if op == 'VEL':
                        velocity = values
                        velocity_until = time.monotonic() + duration
                        last_vel = time.monotonic()
                except (ValueError, OverflowError) as error:
                    print(str(error), file=sys.stderr, flush=True)
        raise TimeoutError('等待执行结果超时')


# 入口只解析 socket、超时和显示/台架选项；异常返回失败码，Ctrl+C 返回 130，不擅自终止原生进程。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=socket_path())
    parser.add_argument(
        '--command', help='single read-only/ground command; flight uses interactive console'
    )
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--json', action='store_true')
    parser.add_argument(
        '--bench-arm-only',
        action='store_true',
        help='allow bench ARM_ONLY interaction with explicit confirmation; NEVER enables server flight gates',
    )
    args = parser.parse_args()
    try:
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        return run(args)
    except (OSError, ValueError, TimeoutError, ConnectionError) as error:
        print(str(error), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
