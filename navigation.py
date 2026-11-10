#!/usr/bin/env python3
"""Management and read-only status view. Flight remains in the native service."""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import pwd
import subprocess
import sys
import time
from navigation_status import (aircraft_feedback_current, controller_state_current, manual_rc_guidance_allowed,
                               preparation_diagnostic_reason, preparation_wait_details,
                               recovery_wait_details)

SERVICE='fc01-navigation.service'
ENTRY=Path(__file__).resolve().parent

def service_info():
    result=subprocess.run(['/usr/bin/systemctl','show',SERVICE,'--property=ActiveState,MainPID,UnitFileState'],
                          capture_output=True,text=True,timeout=4)
    info=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    try:info['boot_id']=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:info['boot_id']=''
    return info

def snapshot(home):
    try:
        row=json.loads((home/'FC01_navigation/status.json').read_text())
        return row if isinstance(row,dict) else {}
    except (OSError,ValueError):return {}

def render_status(row,info,now=None):
    now=time.time() if now is None else now
    state=row.get('state') if isinstance(row.get('state'),dict) else {}
    active=info.get('ActiveState')
    if active not in ('active','activating','deactivating'):
        status='STOPPED';reason='控制服务未运行；历史状态不代表当前飞机状态'
    elif str(row.get('supervisor_pid'))!=info.get('MainPID') or (row.get('boot_id') and info.get('boot_id') and row['boot_id']!=info['boot_id']):
        status='STARTING';reason='等待当前服务进程生成状态'
    elif type(row.get('updated_unix')) not in (float,int) or not math.isfinite(row['updated_unix']) or not 0<=now-row['updated_unix']<=12:
        status='STALE';reason='状态快照过期，当前控制状态未知'
    else:
        status=row.get('status','WAITING');reason=row.get('reason','等待状态')
    age=row.get('state_age_seconds')
    if type(age) not in (int,float) or not math.isfinite(age) or age<0:
        age=None
    updated=row.get('updated_unix')
    if age is not None and type(updated) in (int,float) and math.isfinite(updated):
        age+=max(0,now-updated)
    native_current=(status not in ('STARTING','STALE','STOPPED','STOPPING')
                    and controller_state_current(row,age=age))
    locked_fault=native_current and state.get('phase')=='SAFETY'
    current=(native_current and row.get('feedback_current') is True
             and age is not None and age<=3 and state.get('status_fresh') is True
             and state.get('local_fresh') is True and aircraft_feedback_current(state,age))
    if locked_fault:
        status='FAULT';reason=state.get('reason') or reason
    elif not current and status not in ('STARTING','STALE','STOPPED','STOPPING','WAITING'):
        reason=f'飞机反馈已过期或无效；最近状态 {status}；{reason}'
        status='STALE'
    if current and status=='HEIGHT_HOLD' and state.get('liftoff_confirmed') is False:
        status='LIFTOFF_UNCONFIRMED'
        reason='离地未确认：'+str(state.get('liftoff_unconfirmed_reason','反馈证据未满足'))
    diagnostic=preparation_diagnostic_reason(state)
    if current and diagnostic and status not in ('FAULT','STOPPING'):
        status='DIAGNOSTIC';reason=diagnostic
    mode={15:'Stabilized',14:'Offboard',1:'Altitude'}.get(state.get('nav_state'),str(state.get('nav_state','未知（未收到 nav_state）')))
    armed=state.get('armed')
    armed_text=str(armed) if type(armed) is bool else '未知（未收到有效 armed）'
    height=state.get('relative_height_m')
    height_reason=state.get('height_unavailable_reason') or '未收到有效 relative_height_m'
    height_text=f'{height:.2f} m' if type(height) in (float,int) and math.isfinite(height) else f'未知（{height_reason}）'
    lines=[f'FC01 导航控制  [{status}]',f'原因：{reason}']
    if state.get('phase')=='SAFETY':
        label='故障锁存（当前控制器）' if locked_fault else '最近故障锁存（历史）'
        lines.append(f"{label}：{state.get('reason','未知')}")
        if locked_fault and not current:
            lines.append('飞控反馈：已过期或无效；当前飞机状态未知，不据此解锁或进入导航。')
    if current:
        lines.append(f'实际模式：{mode}  armed={armed_text}  相对高度：{height_text}  反馈年龄：{age:.1f} 秒')
    elif state:
        age_text=f'{age:.1f} 秒前' if age is not None else '接收时间未知'
        lines.append(f'最近反馈（历史，{age_text}；当前飞机状态未知）：模式 {mode}  armed={armed_text}  相对高度：{height_text}')
    else:
        lines.append('当前飞机状态未知：尚未收到飞机反馈；模式、armed 和相对高度均未知。')
    if state:
        gate=lambda name: str(state[name]).lower() if type(state.get(name)) is bool else '未知'
        gate_label='原生动作门' if current else '最近动作门（历史）'
        lines.append(f"{gate_label}：flight_actions_enabled={gate('flight_actions_enabled')}  takeoff_enabled={gate('takeoff_enabled')}  arm_enabled={gate('arm_enabled')}（程序解锁）  flight_enabled={gate('flight_enabled')}（通用控制入口）")
        if status=='DIAGNOSTIC':
            lines.append(f"内部阶段：{state.get('phase','未知')}；原生原因：{state.get('reason','未知')}")
        lines.append(f"起飞平台：{state.get('rc_target_height','未知')} m  心跳预发送提交：{state.get('rc_prestream_commits',0)}  日志丢弃：{state.get('native_log_dropped',0)}")
        lines.append(f"导航档位：期望 {state.get('rc_navigation_mode_slot','未知')} / 观测 {state.get('rc_mode_slot','未知')}")
        if native_current:
            preparation=preparation_wait_details(state)
            if preparation:
                lines.append('准备等待：'+'；'.join(preparation))
        if (state.get('rc_first_fault_reason') or state.get('rc_latest_fault_reason') or
            state.get('phase')=='SAFETY' or status=='FAULT' or state.get('rc_recovery_window_active') is True):
            label=('恢复观察' if current else '恢复观察（飞控反馈过期）') if native_current else '最近恢复观察（历史）'
            first=state.get('rc_first_fault_reason') or '未记录'
            latest=state.get('rc_latest_fault_reason') or '未记录'
            lines.append(f'{label}：首次故障 {first}；最新故障 {latest}')
            runtime_block=state.get('rc_reprepare_block_reason')
            core_block=state.get('rc_recovery_block_reason')
            blocked=runtime_block
            if not blocked or blocked=='RC_CORE_RESTART_CONDITIONS_NOT_MET':blocked=core_block or blocked
            blocked=blocked or '未报告阻塞'
            lines.append(f'恢复等待原因：{blocked}')
            if native_current:
                observations=recovery_wait_details(state)
                if observations:
                    snapshot_age=f'（{age:.1f} 秒前）' if age is not None else ''
                    lines.append('恢复条件快照'+snapshot_age+'：'+'；'.join(observations))
            elapsed=state.get('rc_recovery_elapsed_s');required=state.get('rc_recovery_required_s')
            valid_seconds=lambda value: type(value) in (int,float) and math.isfinite(value) and value>=0
            window=(f'{elapsed:.2f}/{required:.2f} 秒' if valid_seconds(elapsed) and valid_seconds(required) else '未知')
            counts=' / '.join(str(state.get('rc_recovery_'+name+'_samples','未知')) for name in ('status','land','local'))
            lines.append(f'恢复地面窗口：{window}；新样本 status / land / local：{counts}')
            lines.append(f"恢复证据就绪={gate('rc_recovery_ready')}  本轮不安全事件={gate('rc_recovery_event_blocked')}  故障允许恢复={gate('rc_fault_recoverable')}")
        if state.get('takeoff_stage'):
            liftoff='本轮曾通过离地反馈确认，实际离地需现场核对' if state.get('liftoff_confirmed') is True else '离地未确认'
            navigation='导航已允许' if state.get('navigation_allowed') is True else '导航未允许'
            lines.append(f"阶段：{state['takeoff_stage']}；{liftoff}；{navigation}")
            if state.get('liftoff_confirmed') is not True:
                lines.append('离地等待：'+str(state.get('liftoff_unconfirmed_reason','未知')))
        thrust=state.get('last_target_local_thrust_z')
        if type(thrust) in (int,float) and math.isfinite(thrust):
            lines.append(f'本地推力 FRD z：{thrust:.3f}；PX4 采用尚未取证（需对应飞控记录）')
    if row.get('retry_in_seconds') is not None:
        lines.append(f"重试：{row['retry_in_seconds']} 秒后")
    error=row.get('ipc_error')
    if isinstance(error,dict):
        lines.append(f"IPC：{error.get('kind','未知')}；{error.get('message','')}")
    if row.get('controller_exit_reason'):
        lines.append('控制器：'+str(row['controller_exit_reason']))
    guidance={'READY_WAIT_ARM':'已准备：CH5 手动解锁，随后在 8 秒内将 CH6 拨入导航。',
              'WAIT_NAVIGATION':'等待真实解锁/导航证据；保持最低油门，按提示拨 CH6。',
              'DIAGNOSTIC':'仅进行软件诊断；保持未解锁，不执行起飞或导航。',
              'FAULT':'任务已停止推进；按遥控器实际模式接管，接地上锁后复位 CH5/CH6。',
              'HEIGHT_HOLD':'正在定高；任务总时限 30 秒后进入原生降落流程。'}
    if status in ('READY_WAIT_ARM','WAIT_NAVIGATION') and not manual_rc_guidance_allowed(state):
        guidance[status]='尚未确认飞行动作许可；不据此解锁、起飞或进入导航。'
    lines.append(guidance.get(status,'起飞前：Stabilized、最低油门、CH5 未解锁、CH6 非导航、CH7 正常。')
                 if current else '请以飞控和遥控器的实际反馈确认飞机状态。')
    lines.append('窗口只显示状态。关闭窗口/Ctrl+C 不会停止控制服务。')
    return '\n'.join(lines)

def monitor(home):
    info={};next_info=0
    try:
        while True:
            if time.monotonic()>=next_info:
                info=service_info();next_info=time.monotonic()+3
            if sys.stdout.isatty():print('\033[2J\033[H',end='')
            print(render_status(snapshot(home),info),flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        try:print('\n已退出状态查看；后台控制服务保持运行。')
        except OSError:pass
    except (OSError,ValueError):
        return  # Vanished view output never becomes a service stop request.

def report(home):
    from navigation_report import collect_report
    return collect_report(home,entry=ENTRY,runner=subprocess.run)

def main(argv=None):
    parser=argparse.ArgumentParser(description='FC01 手动解锁、遥控触发导航定高')
    parser.add_argument('action',nargs='?',default='install',choices=('install','status','monitor','logs','report','start','stop','land','disable','rollback'))
    parser.add_argument('--mode',choices=('diagnostic','flight-test'),default='diagnostic')
    parser.add_argument('--confirm-flight-test',action='store_true')
    args=parser.parse_args(argv);os.umask(0o077)
    if args.mode=='flight-test' and not args.confirm_flight_test:
        parser.error('Flight Test 必须同时提供 --confirm-flight-test')
    home=Path(pwd.getpwuid(os.getuid()).pw_dir)
    try:
        if args.action in ('install','rollback'):
            from navigation_install import install,rollback
            if args.action=='install':install(ENTRY,home,args.mode,args.confirm_flight_test)
            else:rollback(ENTRY,home)
            if args.action=='install' and sys.stdin.isatty():monitor(home)
        elif args.action=='monitor':monitor(home)
        elif args.action=='status':print(render_status(snapshot(home),service_info()))
        elif args.action=='report':report(home)
        elif args.action=='logs':
            subprocess.run(['/usr/bin/journalctl','-u',SERVICE,'-n','60','--no-pager'],check=False)
            for name in ('standby.log','controller.log'):
                p=home/'FC01_navigation'/name
                if p.is_file():
                    with open(p,'rb') as f:f.seek(max(0,p.stat().st_size-16000));print(name+'\n'+f.read().decode(errors='replace'))
        else:
            op={'start':['start'],'stop':['stop'],'land':['stop'],'disable':['disable','--now']}[args.action]
            if args.action in ('stop','land','disable'):print('请求原生控制器降落/释放并等待安全退出；不强杀。',flush=True)
            subprocess.run(['sudo','/usr/bin/systemctl',*op,SERVICE],check=True)
            print(render_status(snapshot(home),service_info()))
        return 0
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
        print('未完成：'+str(exc),flush=True)
        try:report(home)
        except OSError as error:print('报告写入失败：'+str(error),flush=True)
        return 1

if __name__=='__main__':raise SystemExit(main())
