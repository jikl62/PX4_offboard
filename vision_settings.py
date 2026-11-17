# 视觉参数合同：把开发 JSON 转成 ROS 参数，但不提供飞行权限。
# 时间单位为秒；速度/加速度为 m/s 与 m/s²，角速度 rad/s，倾角度数，高度和半径为米。
"""Development-only visual navigation settings; existing flight tuning is retained."""
import json
import math
from pathlib import Path
import re

from launcher import ControlError

# 这些是视觉外环的候选限制，不替代原高度/姿态控制器参数；启用时仍受原包络约束。
DEFAULTS = dict(
    state_timeout=0.25,
    command_timeout=0.30,
    max_measurement_delay=0.20,
    future_tolerance=0.05,
    position_gain=0.8,
    velocity_gain=1.2,
    max_speed=0.5,
    max_acceleration=0.6,
    max_tilt_deg=5.0,
    max_yaw_rate=0.35,
    max_vertical_speed=0.15,
    max_radius=2.0,
    max_height=5.0,
    min_quality=0.5,
    max_position_jump=0.75,
    max_velocity_jump=1.0,
)


# 仅接受明确的绝对 ROS 话题；拒绝 /fmu/ 下的子话题，避免误接到 PX4 控制/反馈入口。
def topic(value):
    if not isinstance(value, str) or not re.fullmatch(
        r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*', value
    ):
        raise ControlError('视觉话题必须为明确的绝对 ROS 话题名')
    if value.startswith('/fmu/'):
        raise ControlError('视觉输入话题不能指向 /fmu 控制/反馈入口')
    return value


# 原地更新目标配置并返回同一对象；字段白名单、精确类型与有限正数检查共同定义输入合同。
# 这不是事务：后项失败时，前项可能已经写入config；调用者必须丢弃失败候选，不把它继续交给节点。
# 导出器先由navigation_profile复制基础配置，再调用本函数；本函数自身不启动ROS，也不写源YAML。
def apply_settings(config, selected):
    if not isinstance(selected, dict) or set(selected) - {
        'enabled',
        'state_topic',
        'command_topic',
        'qos_depth',
        'state_reliable',
        'command_reliable',
        'parameters',
    }:
        raise ControlError('视觉配置含未知字段或不是对象')
    p = config['fc01_system']['ros__parameters']
    # 布尔值必须是真正的 bool，不能把数字、字符串或 Python 的 bool/int 继承关系当作配置授权。
    for key, default in (('enabled', False), ('state_reliable', False), ('command_reliable', True)):
        value = selected.get(key, default)
        if type(value) is not bool:
            raise ControlError('视觉配置 ' + key + ' 必须为布尔值')
        p['visual_navigation_enabled' if key == 'enabled' else 'visual_' + key] = value
    for key, default in (
        ('state_topic', '/fc01/vision/state'),
        ('command_topic', '/fc01/navigation/command'),
    ):
        p['visual_' + key] = topic(selected.get(key, default))
    if p['visual_navigation_enabled'] and p['visual_state_topic'] == p['visual_command_topic']:
        # 两种消息类型不能共用这条输入；对齐生产Node已有的构造拒绝，避免导出成功后才失败。
        raise ControlError('视觉状态与目标指令话题不能相同')
    # QoS 深度限制待接收数量；它不证明每条消息大小、传感器频率或延迟已经满足控制要求。
    depth = selected.get('qos_depth', 5)
    if type(depth) is not int or not 1 <= depth <= 20:
        raise ControlError('视觉 QoS depth 必须为 1–20 整数')
    p['visual_qos_depth'] = depth
    values = selected.get('parameters', {})
    if not isinstance(values, dict) or set(values) - DEFAULTS.keys():
        raise ControlError('视觉控制参数含未知字段')
    for key, default in DEFAULTS.items():
        value = values.get(key, default)
        try:
            invalid = type(value) not in (int, float) or not math.isfinite(value) or value <= 0
        except OverflowError as exc:
            # JSON 整数没有 double 的范围限制；保持配置错误合同，同时保留原转换异常供定位。
            raise ControlError('视觉参数必须为有限正数：' + key) from exc
        if invalid:
            raise ControlError('视觉参数必须为有限正数：' + key)
        p['visual_' + key] = float(value)
    if p['visual_min_quality'] > 1:
        raise ControlError('visual_min_quality 不能大于 1')
    # 只有启用视觉时才比较原控制包络；视觉不能扩大允许的倾角、高度或升降速度。
    if p['visual_navigation_enabled'] and (
        p['visual_max_tilt_deg'] > p['aa_max_tilt_deg']
        or p['visual_max_height'] > p['aa_max_height']
    ):
        raise ControlError('视觉限幅不能扩大原倾角/高度包络')
    if p['visual_navigation_enabled'] and p['visual_max_vertical_speed'] > min(
        p['aa_max_climb'], p['aa_max_descent']
    ):
        raise ControlError('视觉升降速度不能扩大原垂直速度包络')
    return config


# 缺少视觉设置文件时明确保持关闭；存在但损坏则报错，不静默忽略错误配置。
def load(config, entry):
    path = Path(entry) / 'vision_settings.json'
    try:
        selected = json.loads(path.read_text()) if path.exists() else {'enabled': False}
    except (ValueError, OSError) as exc:
        raise ControlError('读取视觉配置失败：' + str(exc)) from exc
    return apply_settings(config, selected)
