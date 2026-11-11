#!/usr/bin/env python3
# 交互任务的纯 Python 合同与单次事务：这里只管理准备顺序，真实飞行由已安装 C++ Runtime 决定。
# 高度单位 m、相对地面向上为正；ground_vz_ned 为 PX4 NED m/s，向下为正。
"""Own the launch/console lifecycle; the installed C++ controller owns flight."""
from __future__ import annotations

import copy
import math


# 配置或操作者请求不满足合同的显式异常，调用入口须报告而非继续推进。
class ControlError(RuntimeError):
    pass


# 原无视觉候选起飞平台，不因视觉接口或代码排版而改变。
DEFAULT_TAKEOFF_HEIGHT_M = 1.5


# 把各控制权和动作许可分开列出；通用速度 flight_enabled 不属于姿态定高入口授权。
ACTION_FLAGS = (
    'flight_enabled', 'flight_actions_enabled', 'arm_enabled', 'takeoff_enabled',
    'flight_parameters_confirmed', 'rc_takeover_confirmed',
    'autonomous_failsafe_confirmed', 'bench_arm_only_enabled',
)


# 深拷贝基础配置后校验结构、实时时钟、数值和已验证 20 Hz/落地采样合同；返回独立候选。
def runtime_profile(base: dict, flight: bool) -> dict:
    out = copy.deepcopy(base)
    try:
        p = out['fc01_system']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise ControlError('缺少 fc01_system/ros__parameters') from exc
    if not isinstance(p, dict) or p.get('profile') != 'attitude_altitude':
        raise ControlError('只接受现有 attitude_altitude 配置')
    if p.get('use_sim_time', False) is not False:
        raise ControlError('真实 FC01 不接受仿真时钟')
    for name, value in p.items():
        if name.startswith('aa_') and isinstance(value, (float, int)):
            if isinstance(value, bool) or not math.isfinite(value):
                # aa_vertical_feedback_guard_enabled is deliberately boolean.
                if not (name == 'aa_vertical_feedback_guard_enabled' and type(value) is bool):
                    raise ControlError('无效数值参数: '+name)
    for name in ('aa_max_height', 'aa_land_timeout', 'aa_land_sample_max_gap',
                 'aa_hover_thrust', 'control_loop_hz'):
        if type(p.get(name)) not in (int, float) or not math.isfinite(p[name]):
            raise ControlError('缺少有效数值参数: '+name)
    if not 0 < p['aa_max_height'] <= 5.0:
        raise ControlError('当前入口的高度硬上限为 5.0 m')
    if p['control_loop_hz'] != 20.0:
        raise ControlError('当前验证基线要求 20 Hz 控制环')
    if not (1.0 < min(p['aa_land_sample_max_gap'], p['aa_land_timeout'] / 2)
            and p['aa_land_sample_max_gap'] < p['aa_land_timeout']):
        raise ControlError('落地采样间隔参数不兼容已测得的约 1 Hz 反馈')
    # 先关闭所有门，再按显式 --flight 选择开独立的原姿态定高门，避免继承文件里的旧授权。
    for key in ACTION_FLAGS:
        p[key] = False
    if flight:
        # --flight is the operator's explicit selection of the displayed
        # existing candidate parameters/RC route, not new calibration evidence.
        # Config::validate() forbids the legacy flight_enabled switch for this
        # profile. Only the dedicated attitude/altitude action gates are used.
        for key in ACTION_FLAGS[1:6]:
            p[key] = True
    return out


# 起飞准备须具备的新鲜度、源时钟、预检、RC和真实地面证据；缺字段等价于未满足。
REQUIRED_TRUE = (
    'status_fresh', 'local_fresh', 'land_fresh', 'attitude_fresh',
    'failsafe_fresh', 'battery_fresh', 'local_sample_fresh',
    'attitude_sample_fresh', 'source_clock_valid', 'source_clock_validated',
    'z_valid', 'vz_valid', 'arming_state_known', 'battery_connected',
    'preflight', 'rc_link_ready', 'rc_switches_ready',
    'rc_source_clock_valid', 'rc_switch_clock_valid', 'landed', 'at_rest',
)
REQUIRED_FALSE = (
    'armed', 'rc_sticks_moving', 'rc_kill_latched',
    'publication_fault_latched', 'operator_takeover_latched',
)


# 返回具体阻塞项而不发命令；严格 bool/有限数、地面持续窗口、NED速度和唯一来源都须满足。
def readiness_issues(state: dict) -> list[str]:
    issues = [key for key in REQUIRED_TRUE if state.get(key) is not True]
    issues += [key for key in REQUIRED_FALSE if state.get(key) is not False]
    for key in ('stationary_seconds', 'required_ground_seconds',
                'ground_vz_ned', 'ground_vz_tolerance'):
        value = state.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            issues.append(key)
    if not any(key in issues for key in ('stationary_seconds', 'required_ground_seconds')):
        if state['required_ground_seconds'] < 2 or state['stationary_seconds'] < state['required_ground_seconds']:
            issues.append('ground_dwell')
    if not any(key in issues for key in ('ground_vz_ned', 'ground_vz_tolerance')):
        if state['ground_vz_tolerance'] <= 0 or abs(state['ground_vz_ned']) > state['ground_vz_tolerance']:
            issues.append('ground_vz')
    counts = state.get('source_counts')
    if not isinstance(counts, list) or len(counts) != 3 or any(type(n) is not int or n != 1 for n in counts):
        issues.append('unique_status_land_ack_publishers')
    return issues


# 单进程内一次用户起飞请求的状态机；序列号关联回复且不重试 TAKEOFF，不替代原生安全门。
class Mission:
    """A user-issued takeoff is one sequenced, non-retrying transaction."""

    # state/state_at 为最后快照及主机单调接收时间；pending 是待回复序列；submitted 永久记录本进程已提交。
    def __init__(self, max_height: float, prepare_timeout: float = 25.0):
        self.max_height = max_height
        self.prepare_timeout = prepare_timeout
        self.state: dict = {}
        self.state_at = -math.inf
        self.stage = 'IDLE'
        self.target = 0.0
        self.deadline = 0.0
        self.pending: tuple[int, str] | None = None
        self.prepared = False
        self.submitted = False
        self.error: str | None = None

    @property
    # 仅准备/等待中阶段算活动；DONE 代表请求已交给原生控制器，不是证明飞机已完成起飞。
    def active(self) -> bool:
        return self.stage not in ('IDLE', 'DONE', 'FAILED')

    # 保存新快照；活动任务看到安全锁存/接管或准备期意外已解锁就停止推进。
    def observe(self, state: dict, now: float) -> None:
        self.state = state
        self.state_at = now
        if self.active and (state.get('phase') == 'SAFETY' or
                any(state.get(k) is True for k in
                    ('rc_kill_latched', 'operator_takeover_latched', 'publication_fault_latched'))):
            self.fail('控制器保护/接管: '+str(state.get('reason', state.get('phase'))))
        elif self.active and not self.submitted and state.get('armed') is True:
            self.fail('准备阶段观察到已解锁；未提交新的 TAKEOFF')

    # 校验目标和本进程一次性规则；只有未解锁 IDLE/OFFBOARD_READY 可以开始等待准备证据。
    def request_takeoff(self, height: float, now: float) -> None:
        if type(height) not in (int, float) or not math.isfinite(height) or not 0 < height < self.max_height:
            raise ControlError('高度必须为有限数，且在 (0, %.2f) m 内' % self.max_height)
        if self.submitted:
            raise ControlError('本进程已经提交过 TAKEOFF；禁止自动或重复解锁')
        if self.active:
            raise ControlError('起飞准备仍在进行')
        if self.state.get('armed') is True or self.state.get('phase') not in ('IDLE', 'OFFBOARD_READY'):
            raise ControlError('需要当前未解锁的 IDLE/OFFBOARD_READY 状态')
        self.target = float(height)
        self.deadline = now + self.prepare_timeout
        self.prepared = False
        self.pending = None
        self.error = None
        self.stage = 'WAIT_READY' if self.state.get('phase') == 'OFFBOARD_READY' else 'WAIT_GROUND'

    # 只取消 Python 准备事务，不清除 submitted 一次性水位，也不假装撤销已经发送的飞行任务。
    def cancel(self) -> None:
        self.stage = 'IDLE'
        self.pending = None

    # 保存明确错误并清待回复状态；后续正常状态不能静默重新发起本次任务。
    def fail(self, reason: str) -> None:
        self.error = reason
        self.stage = 'FAILED'
        self.pending = None

    # 先标记将发出的步骤，实际发送后必须用 sent 关联序列号，避免绕过推进顺序。
    def issue(self, op: str, values: list) -> tuple[str, list]:
        self.stage = 'ISSUED_'+op
        return op, values

    # 单调时间到期时已准备者请求 LAND/释放；状态过期/不完整则只等待，不以旧快照发动作。
    def poll(self, now: float) -> tuple[str, list] | None:
        if not self.active or self.submitted:
            return None
        if now >= self.deadline:
            release = self.prepared
            self.fail('PREPARATION_TIMEOUT；未提交 TAKEOFF')
            return ('LAND', []) if release else None
        if now - self.state_at > 1.0 or now < self.state_at:
            return None
        if readiness_issues(self.state):
            return None
        if self.stage == 'WAIT_GROUND' and self.state.get('phase') == 'IDLE' and self.state.get('nav_state') != 14:
            return self.issue('AUTHORITY_RC', [])
        if self.stage == 'PREPARE' and self.state.get('phase') == 'IDLE':
            return self.issue('ATTITUDE_READY', [])
        # 只有明确 Offboard、RC接管权限已锁定且符合原176 ACK+状态策略，才允许唯一一次 TAKEOFF。
        if self.stage == 'WAIT_READY':
            required = {'phase': 'OFFBOARD_READY', 'nav_state': 14,
                        'authority_mode': 'RC_TAKEOVER', 'authority_attempt_mode': 'RC_TAKEOVER',
                        'authority_ready': True, 'authority_locked': True,
                        'm1_176_ack_accepted': True,
                        'm1_176_ack_policy': 'R632_ACK176_STATE_CANDIDATE_V1'}
            if all(type(self.state.get(k)) is type(v) and self.state[k] == v for k, v in required.items()):
                self.submitted = True
                return self.issue('TAKEOFF', [self.target])
        return None

    # 发送完成后记录 seq/op，接下来只接受这一笔 reply；ATTITUDE_READY 已提交后需要保留释放责任。
    def sent(self, seq: int, op: str) -> None:
        if self.stage != 'ISSUED_'+op:
            raise ControlError('未授权的任务推进: '+op)
        self.pending = (seq, op)
        self.stage = 'WAIT_REPLY_'+op
        if op == 'ATTITUDE_READY':
            self.prepared = True

    # 回复必须是 pending 的整数序号；拒绝明确失败，成功只推进既定 AUTHORITY→READY→TAKEOFF 链。
    def reply(self, obj: dict, now: float) -> None:
        if (not self.pending or obj.get('kind') != 'reply' or
                type(obj.get('seq')) is not int or obj['seq'] != self.pending[0]):
            return
        op = self.pending[1]
        self.pending = None
        if obj.get('accepted') is not True:
            self.fail('控制器拒绝 '+op+': '+str(obj.get('message', 'invalid reply')))
            return
        self.stage = {'AUTHORITY_RC': 'PREPARE', 'ATTITUDE_READY': 'WAIT_READY', 'TAKEOFF': 'DONE'}[op]
