#!/usr/bin/env python3
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

from fc01_control import ControlError, DEFAULT_TAKEOFF_HEIGHT_M, runtime_profile
from launcher import PROJECT, private_dir, verify_binary
import standby

ENTRY = Path(__file__).resolve().parent
MARKER = b'RC_NAVIGATION_MANUAL_ARM_V1'
TAKEOFF_MARKER = b'NAV_TAKEOFF_EVIDENCE_V2_20261103'

def navigation_profile(base, target=DEFAULT_TAKEOFF_HEIGHT_M, mode_slot=6):
    config = runtime_profile(base, False)
    p = config['fc01_system']['ros__parameters']
    if type(target) not in (int,float) or not math.isfinite(target) or not 0 < target < p['aa_max_height']:
        raise ControlError('导航高度必须为有限数字且位于 (0, aa_max_height)')
    if p['aa_max_tilt_deg'] > 10 or p['aa_max_flight_seconds'] > 30:
        raise ControlError('导航沿用最大 10 度/30 秒限制')
    if type(mode_slot) is not int or not 1 <= mode_slot <= 6:
        raise ControlError('导航 mode_slot 必须是 1–6 的整数；对应飞控模式档位，不是 CH6 通道号')
    p.update(rc_navigation_enabled=True, rc_target_height=float(target), rc_navigation_mode_slot=mode_slot,
             rc_arm_to_navigation_timeout=8., flight_enabled=False, flight_actions_enabled=True,
             arm_enabled=False, takeoff_enabled=True, flight_parameters_confirmed=True,
             rc_takeover_confirmed=True, autonomous_failsafe_confirmed=False,
             bench_arm_only_enabled=False, csv_path='')
    return config

def settings(entry=ENTRY):
    path=entry/'navigation_settings.json'
    return json.loads(path.read_text()) if path.exists() else {'target_height_m':DEFAULT_TAKEOFF_HEIGHT_M}

def controller_path(entry=ENTRY,prefix=None):
    return (prefix or entry/'install')/'lib/jikl_fc01_system/fc01_system_node'

def check_controller(entry=ENTRY,prefix=None):
    binary=controller_path(entry,prefix)
    verify_binary(binary)
    if MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少手动解锁导航标识，保留原服务')
    if TAKEOFF_MARKER not in binary.read_bytes():
        raise ControlError('可执行程序缺少本版起飞证据标识，请重新构建；保留原服务')
    return binary

def load_profile(root,agent,entry=ENTRY):
    base=standby.validate(root,agent)
    selected=settings(entry)
    slot=selected.get('navigation_mode_slot',base['fc01_system']['ros__parameters'].get('rc_navigation_mode_slot',6))
    config=navigation_profile(base,selected.get('target_height_m',DEFAULT_TAKEOFF_HEIGHT_M),slot)
    from vision_settings import load
    return load(config,entry)

def describe_row(row):
    state=row.get('state') or {}
    phase=state.get('phase')
    result=dict(row)
    if row.get('feedback_current') is False or row.get('status') in ('STOPPING','STOPPED') or not phase:
        return result
    reason=state.get('reason') or row.get('reason','')
    if phase=='SAFETY':
        result.update(status='FAULT',reason=reason)
    elif row.get('status')=='CONNECTED':
        names={'RC_PRESTREAM':'PRESTREAM','RC_READY_WAIT_ARM':'READY_WAIT_ARM',
               'RC_ARMED_WAIT_NAVIGATION':'WAIT_NAVIGATION','ARMED_GROUND':'ARMED_GROUND',
               'THRUST_RAMP':'THRUST_RAMP','ASCEND':'ASCEND','HEIGHT_HOLD':'HEIGHT_HOLD',
               'DESCENT':'LANDING','RECOVERY_DESCENT':'LANDING','POWERED_GROUND_UNCONFIRMED':'LANDING',
               'LANDED':'LANDING','REQUEST_DISARM':'WAIT_DISARM','SAFE_DISARMED':'SAFE_DISARMED',
               'RELEASE_OFFBOARD':'RELEASING'}
        result.update(status=names.get(phase,'WAITING'),reason=reason)
    else:
        result.update(status='WAITING',reason=reason)
    return result

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('run','prepare-config','validate-env','check-config','probe','build'),nargs='?',default='run')
    parser.add_argument('--file',type=Path)
    parser.add_argument('--prefix',type=Path)
    parser.add_argument('--build-dir',type=Path)
    args=parser.parse_args(argv)
    os.umask(0o077)
    root=Path(os.environ.get('FC01_PROJECT_ROOT',str(Path.home()/PROJECT))).expanduser().resolve()
    agent=Path.home()/'fc01_agent_build_offline/MicroXRCEAgent'
    if args.action=='validate-env':
        load_profile(root,agent)
        print('ROS / 消息 / 原生工程 / 手动解锁导航参数检查通过。')
    elif args.action=='prepare-config':
        import yaml
        if args.file is None:parser.error('prepare-config requires --file')
        private_dir(args.file.parent)
        args.file.write_text(yaml.safe_dump(load_profile(root,agent),allow_unicode=True,sort_keys=False))
    elif args.action=='build':
        if args.prefix is None or args.build_dir is None:parser.error('build requires --prefix and --build-dir')
        from navigation_install import build_native
        build_native(ENTRY,args.prefix,args.build_dir)
    elif args.action=='check-config':
        if args.file is None:parser.error('check-config requires --file')
        check_controller(prefix=args.prefix)
        checker=(args.prefix or ENTRY/'install')/'lib/jikl_fc01_system/fc01_config_check'
        if not checker.is_file() or not os.access(checker,os.X_OK):
            raise ControlError('缺少已编译的原生配置检查器')
        return subprocess.run([str(checker),'--ros-args','--disable-external-lib-logs',
                               '--params-file',str(args.file)],check=False).returncode
    elif args.action=='probe':
        load_profile(root,agent)
        from navigation_probe import probe
        return probe(root,agent,args.file)
    else:
        stopped=threading.Event()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            signal.signal(sig,lambda *_:stopped.set())
        standby.supervise(root,agent,Path.home()/'FC01_navigation',stopped,
            controller=controller_path(),config_factory=lambda r,a:(check_controller(),load_profile(r,a))[1],
            status_metadata={'rc_navigation_enabled':True,'flight_actions_enabled':True,'automatic_arm':False},
            row_formatter=describe_row)
    return 0

if __name__=='__main__':
    raise SystemExit(main())
