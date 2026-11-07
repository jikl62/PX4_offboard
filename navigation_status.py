"""Pure display rules for native preparation and its actual action gates."""

PREPARATION_PHASES = frozenset(('IDLE', 'SELF_CHECK', 'PRESTREAM', 'REQUEST_OFFBOARD',
                               'RC_PRESTREAM', 'RC_READY_WAIT_ARM', 'RC_ARMED_WAIT_NAVIGATION'))


def preparation_diagnostic_reason(state):
    """A native phase describes feedback progress, not permission to take off."""
    if state.get('phase') not in PREPARATION_PHASES:
        return ''
    closed = [name+'=false' for name in ('flight_actions_enabled', 'takeoff_enabled')
              if state.get(name) is False]
    return '诊断模式：'+'、'.join(closed)+'；本次禁止起飞和导航。' if closed else ''


def manual_rc_guidance_allowed(state):
    # arm_enabled=false disables programmatic arming, and is required by the
    # independent manual-RC route. flight_enabled belongs to another profile.
    return all(state.get(name) is True for name in
               ('flight_actions_enabled', 'takeoff_enabled', 'rc_navigation_enabled'))
