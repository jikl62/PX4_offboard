# 导航状态窗口的纯显示规则：区分进程/IPC活性、原生状态新鲜度和各飞控话题新鲜度。
# 任何中文提示都不授予准备、恢复或起飞权限，真正决定仍在 C++ 控制器。
"""Pure display rules for native preparation and its actual action gates."""

import math

# 只对这些原生准备阶段解释地面等待；故障阶段由独立恢复解释器处理。
PREPARATION_PHASES = frozenset(
    (
        'IDLE',
        'SELF_CHECK',
        'PRESTREAM',
        'REQUEST_OFFBOARD',
        'RC_PRESTREAM',
        'RC_READY_WAIT_ARM',
        'RC_ARMED_WAIT_NAVIGATION',
    )
)


# 同一boot/控制PID与足够新的IPC快照才可作为当前控制器证据；不表示飞机传感器仍新鲜。
def controller_state_current(row, *, age=None):
    """IPC progress stays distinct from the freshness of the aircraft topics."""
    state = row.get('state')
    if not isinstance(state, dict) or not state:
        return False
    if row.get('status') in ('STARTING', 'STALE', 'STOPPING', 'STOPPED'):
        return False
    if (
        row.get('ipc_error')
        or row.get('controller_returncode') is not None
        or row.get('retry_in_seconds') is not None
    ):
        return False
    controller_pid, state_pid = row.get('controller_pid'), row.get('state_controller_pid')
    if (controller_pid is not None or state_pid is not None) and controller_pid != state_pid:
        return False
    if controller_pid is not None and state.get('controller_pid', controller_pid) != controller_pid:
        return False
    age = row.get('state_age_seconds') if age is None else age
    return type(age) in (int, float) and math.isfinite(age) and 0 <= age <= 3


# 把读取快照后经过的时间加到各话题自身年龄；不能用新显示刷新旧测量的TTL。
def aircraft_feedback_current(state, elapsed):
    """Advance native sample ages without extending their reported lifetimes."""
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    if not finite(elapsed) or elapsed < 0:
        return False
    if any(
        state.get(name) is False
        for name in (
            'status_fresh',
            'land_fresh',
            'local_fresh',
            'attitude_fresh',
            'local_sample_fresh',
            'attitude_sample_fresh',
            'battery_fresh',
            'source_clock_valid',
        )
    ):
        return False
    timings = state.get('source_timing')
    if not isinstance(timings, dict):
        return False
    # 每个关键反馈分别检查 envelope/receipt，局部位置和姿态还检查原样本及样本推进年龄。
    for topic in ('status', 'land', 'local', 'attitude', 'flags', 'battery'):
        item = timings.get(topic)
        if not isinstance(item, dict):
            return False
        timeout = item.get('timeout_s')
        if (
            not finite(timeout)
            or timeout <= 0
            or item.get('source_reason') != 'VALID'
            or item.get('receipt_reason') != 'VALID'
        ):
            return False
        names = ('envelope_age_s', 'receipt_age_s')
        if topic in ('local', 'attitude'):
            names += ('sample_age_s', 'sample_progress_age_s')
        for name in names:
            age = item.get(name)
            if not finite(age) or (name in ('receipt_age_s', 'sample_progress_age_s') and age < 0):
                return False
            # Native VALID may include a source timestamp slightly ahead of
            # the host clock. It must not buy extra display lifetime.
            if max(0.0, age) + elapsed >= timeout:
                return False
    return True


# 根据明确观测列出等待原因；类型未知时不猜测档位、解锁或地面状态。
def preparation_wait_details(state):
    """Explain observed blockers; native code alone decides admission."""
    if state.get('phase') not in PREPARATION_PHASES:
        return []
    details = []
    for name, text in (
        ('status_fresh', '飞控状态反馈过期'),
        ('land_fresh', '接地反馈过期'),
        ('local_fresh', '位置/垂直速度反馈过期'),
        ('attitude_fresh', '姿态反馈过期'),
        ('source_clock_valid', '源消息时间戳过期或无效'),
        ('rc_link_ready', '遥控源/链路未就绪'),
        ('rc_switches_ready', '遥控开关反馈未就绪'),
        ('preflight', '飞控预检未通过'),
    ):
        if state.get(name) is False:
            details.append(text)
    for topic, label in (('local', '位置/垂直速度'), ('attitude', '姿态')):
        if state.get(topic + '_sample_fresh') is False and state.get(topic + '_fresh') is not False:
            details.append(label + '采样过期')
    if state.get('battery_fresh') is False or state.get('battery_connected') is False:
        details.append('电池反馈未就绪')
    if state.get('rc_axes_safe') is False or state.get('rc_sticks_moving') is True:
        details.append('油门/摇杆未归位')
    reason = state.get('reason', '')
    ground_wait = reason in (
        'RC_WAIT_DISARMED_STABILIZED_ARM_OFF_NAV_OFF_GROUND',
        'RC_WAIT_CONTINUOUS_SAFE_GROUND',
        'RC_WAIT_SAFE_GROUND',
        'RC_PRESTREAM_GROUND_CHANGED',
        'RC_READY_GROUND_CHANGED',
    )
    # 仅对已知地面等待原因解释Stabilized、CH5上锁、CH6有效非导航档和连续静止窗口。
    if ground_wait:
        for name, expected, text in (
            ('armed', False, '飞控仍解锁'),
            ('landed', True, '未确认接地'),
            ('at_rest', True, '未确认静止'),
        ):
            if type(state.get(name)) is bool and state[name] is not expected:
                details.append(text)
        if type(state.get('nav_state')) is int and state['nav_state'] != 15:
            details.append('未处于 Stabilized')
        if type(state.get('rc_arm_switch')) is int and state['rc_arm_switch'] != 3:
            details.append('CH5 未复位至上锁')
        slot, navigation_slot = state.get('rc_mode_slot'), state.get('rc_navigation_mode_slot')
        if type(slot) is int and (not 1 <= slot <= 6 or slot == navigation_slot):
            details.append('CH6 未处于有效非导航档')
        elapsed, required = state.get('stationary_seconds'), state.get('required_ground_seconds')
        finite = lambda value: type(value) in (int, float) and math.isfinite(value)
        if finite(elapsed) and finite(required) and elapsed < required:
            details.append(f'持续静止不足（{elapsed:.2f}/{required:.2f} 秒）')
        vz, limit = state.get('ground_vz_ned'), state.get('ground_vz_tolerance')
        if finite(vz) and finite(limit) and abs(vz) > limit:
            details.append(f'垂直速度超出地面阈值（{vz:.3f} / ±{limit:.3f} m/s）')
        if not details:
            details.append('等待原生地面/遥控条件确认；本次状态未指明缺失项')
    elif reason in (
        'RC_PRESTREAM_WAIT_NEW_GROUND_REFERENCE',
        'RC_PRESTREAM_REBUILD_GROUND_REFERENCE',
    ):
        samples, elapsed = state.get('rc_ground_reference_samples'), state.get(
            'rc_ground_reference_seconds'
        )
        seconds = (
            f'{elapsed:.2f} 秒'
            if type(elapsed) in (int, float) and math.isfinite(elapsed)
            else '时间未知'
        )
        details.append(
            f'等待新的地面参考样本（{samples if type(samples) is int else "数量未知"}，{seconds}）'
        )
    return details


# 对故障快照按话题列举过期及独立新鲜落地/状态证据；某路过期不能掩盖另一新鲜不安全观测。
def recovery_wait_details(state):
    """Explain observations in the native snapshot, without granting recovery."""
    if state.get('phase') != 'SAFETY':
        return []
    details = []
    finite = lambda value: type(value) in (int, float) and math.isfinite(value) and value >= 0
    timings = state.get('source_timing')
    if isinstance(timings, dict):
        for topic, label in (
            ('local', '位置/垂直速度'),
            ('attitude', '姿态'),
            ('status', '飞控状态'),
            ('land', '接地'),
            ('flags', '失效保护'),
            ('battery', '电池'),
        ):
            item = timings.get(topic)
            if not isinstance(item, dict) or item.get('source_reason') not in (
                'ENVELOPE_EXPIRED',
                'SAMPLE_EXPIRED',
            ):
                continue
            ages = [item.get(name) for name in ('envelope_age_s', 'sample_age_s')]
            ages = [value for value in ages if finite(value)]
            limit = item.get('timeout_s')
            text = label + '消息过期'
            if ages and finite(limit) and limit > 0:
                text += f'（消息年龄 {max(ages):.3f} 秒，阈值 {limit:.3f} 秒）'
            details.append(text)
            queued = item.get('rmw_to_callback_s')
            if finite(queued):
                details.append(f'{label}中间件接收至回调间隔 {queued:.3f} 秒')
    # Check each topic's freshness independently: a local timeout must not
    # hide a fresh detector saying the aircraft still is not at rest.
    if state.get('land_fresh') is True:
        if state.get('landed') is False:
            details.append('飞控尚未确认接地（landed=false）')
        if state.get('at_rest') is False:
            details.append('飞控尚未确认静止（at_rest=false）')
    if state.get('status_fresh') is True:
        if state.get('armed') is True:
            details.append('飞控仍解锁')
        if state.get('px4_user_takeover_reported') is True:
            details.append('飞控仍报告人工接管有效')
    return details


# 原生准备阶段不等于飞行授权；明确关闭的动作门应显示为诊断禁止起飞。
def preparation_diagnostic_reason(state):
    """A native phase describes feedback progress, not permission to take off."""
    if state.get('phase') not in PREPARATION_PHASES:
        return ''
    closed = [
        name + '=false'
        for name in ('flight_actions_enabled', 'takeoff_enabled')
        if state.get(name) is False
    ]
    return '诊断模式：' + '、'.join(closed) + '；本次禁止起飞和导航。' if closed else ''


# 只根据RC路线三门提示人工步骤；arm_enabled=false是人工解锁合同，不应被误认为故障。
def manual_rc_guidance_allowed(state):
    # arm_enabled=false disables programmatic arming, and is required by the
    # independent manual-RC route. flight_enabled belongs to another profile.
    return all(
        state.get(name) is True
        for name in ('flight_actions_enabled', 'takeoff_enabled', 'rc_navigation_enabled')
    )
