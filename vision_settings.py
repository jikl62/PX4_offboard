"""Development-only visual navigation settings; existing flight tuning is retained."""
import json
import math
from pathlib import Path
import re

from launcher import ControlError

DEFAULTS=dict(state_timeout=.25,command_timeout=.30,max_measurement_delay=.20,future_tolerance=.05,
              position_gain=.8,velocity_gain=1.2,max_speed=.5,max_acceleration=.6,max_tilt_deg=5.,
              max_yaw_rate=.35,max_vertical_speed=.15,max_radius=2.,max_height=5.,min_quality=.5,
              max_position_jump=.75,max_velocity_jump=1.)


def topic(value):
    if not isinstance(value,str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*',value):
        raise ControlError('视觉话题必须为明确的绝对 ROS 话题名')
    if value.startswith('/fmu/'):
        raise ControlError('视觉输入话题不能指向 /fmu 控制/反馈入口')
    return value


def apply_settings(config,selected):
    if not isinstance(selected,dict) or set(selected)-{'enabled','state_topic','command_topic','qos_depth','state_reliable','command_reliable','parameters'}:
        raise ControlError('视觉配置含未知字段或不是对象')
    p=config['fc01_system']['ros__parameters']
    for key,default in (('enabled',False),('state_reliable',False),('command_reliable',True)):
        value=selected.get(key,default)
        if type(value) is not bool:raise ControlError('视觉配置 '+key+' 必须为布尔值')
        p['visual_navigation_enabled' if key=='enabled' else 'visual_'+key]=value
    for key,default in (('state_topic','/fc01/vision/state'),('command_topic','/fc01/navigation/command')):
        p['visual_'+key]=topic(selected.get(key,default))
    depth=selected.get('qos_depth',5)
    if type(depth) is not int or not 1<=depth<=20:raise ControlError('视觉 QoS depth 必须为 1–20 整数')
    p['visual_qos_depth']=depth
    values=selected.get('parameters',{})
    if not isinstance(values,dict) or set(values)-DEFAULTS.keys():raise ControlError('视觉控制参数含未知字段')
    for key,default in DEFAULTS.items():
        value=values.get(key,default)
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ControlError('视觉参数必须为有限正数：'+key)
        p['visual_'+key]=float(value)
    if p['visual_min_quality']>1:raise ControlError('visual_min_quality 不能大于 1')
    if p['visual_navigation_enabled'] and (p['visual_max_tilt_deg']>p['aa_max_tilt_deg'] or p['visual_max_height']>p['aa_max_height']):
        raise ControlError('视觉限幅不能扩大原倾角/高度包络')
    if p['visual_navigation_enabled'] and p['visual_max_vertical_speed']>min(p['aa_max_climb'],p['aa_max_descent']):
        raise ControlError('视觉升降速度不能扩大原垂直速度包络')
    return config


def load(config,entry):
    path=Path(entry)/'vision_settings.json'
    try:selected=json.loads(path.read_text()) if path.exists() else {'enabled':False}
    except (ValueError,OSError) as exc:raise ControlError('读取视觉配置失败：'+str(exc)) from exc
    return apply_settings(config,selected)
