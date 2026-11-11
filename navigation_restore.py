#!/usr/bin/env python3
# 托管导航服务的安全配置/激活/回滚事务：认证后重新取得地面/RC证据，先等旧控制器退出再切配置。
# 保留既有Agent；不强杀节点、不重启失败任务，备份绑定服务/YAML/二进制与开机策略。
"""Configure, safely activate, or restore the managed native navigation service."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import pwd
import shlex
import signal
import subprocess
import tempfile
import time
from datetime import datetime

from launcher import ControlError, choose_agent, preflight, private_dir, process_list, uart_owners
from navigation_install import (MARKER, OLD, OLD_MARKER, SERVICE, boot, managed_unit,
                                render_unit, unit_info)
from navigation_service import ENTRY_VERSION, check_controller

ENTRY = Path(__file__).resolve().parent
UNIT_DIR = Path('/etc/systemd/system')


# 只按实际文件字节计算SHA，供切换前后身份复验。
def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


# 从真实进程表找原生控制器；后续须核对用户和服务父链，不能只凭名字认领。
def _controllers():
    return [p for p in process_list() if any(Path(a).name=='fc01_system_node' for a in p.argv[:2])]


# 沿/proc父链判断归属，遇回环/不可读即False，不把未知进程算进托管服务。
def _descendant(pid, parent, proc=Path('/proc')):
    seen=set()
    while pid>1 and pid not in seen:
        if pid==parent:return True
        seen.add(pid)
        try:
            pid=int(next(s.split()[1] for s in (proc/str(pid)/'status').read_text().splitlines() if s.startswith('PPid:')))
        except (OSError,ValueError,StopIteration):return False
    return False


# 必须恰好复用当前用户同UART的Agent；期望身份变化即拒绝，不创建第二个Agent。
def retained_agent(expected=None):
    agent=choose_agent(process_list(),uart_owners('/dev/ttyS2'),'/dev/ttyS2',os.getuid())
    if agent is None or (expected is not None and agent!=expected):
        raise ControlError('既有 MicroXRCEAgent 缺失或已变化；保留现有进程，不启动其他 Agent')
    return agent


# 探针前后复验Agent并要求新鲜安全RC/地面证据；探针不得启动新Agent。
def _ground(entry, home, project, agent):
    retained_agent(agent)
    try:
        boot(entry,home,project,'probe','--mode','flight-test','--confirm-flight-test',
             '--require-existing-agent','--require-rc-safe',
             '--file',home/'FC01_navigation/probe.json')
    except (RuntimeError,subprocess.SubprocessError) as exc:
        try:reason=json.loads((home/'FC01_navigation/probe.json').read_text()).get('reason','')
        except (OSError,ValueError):reason=str(exc)
        raise ControlError('硬件尚未满足在线启用条件：'+str(reason)+'；详见 '+str(home/'FC01_navigation/install.log')) from exc
    retained_agent(agent)
    from navigation_install import ground_permit_fresh
    if not ground_permit_fresh(home):
        raise ControlError('硬件尚未满足在线启用条件：缺少新鲜未解锁接地证据')
    row=json.loads((home/'FC01_navigation/probe.json').read_text())
    if row.get('agent_pid')!=agent.pid or row.get('agent_created') is not False or row.get('rc_safe') is not True:
        raise ControlError('在线探针未确认保留原 Agent 及真实 RC 安全状态')


# 原子发布本次事务阶段与备份路径，失败恢复可追踪；updated_unix只作诊断身份。
def _activation_state(entry, backup, phase, **extra):
    path=entry/'ACTIVATION_STATE.json'
    row=dict(service=SERVICE,entry=str(entry.resolve()),version=ENTRY_VERSION,
             backup=str(backup.resolve()),phase=phase,updated_unix=time.time(),**extra)
    staged=path.with_suffix('.json.new')
    staged.write_text(json.dumps(row,ensure_ascii=False,indent=2)+'\n')
    os.replace(staged,path)


# 验证托管marker、稳定服务状态和普通开机策略，再核对所有控制器的真实归属。
def _discover(units, run):
    info={name:unit_info(name,run) for name in (SERVICE,OLD)}
    managed_unit(units/SERVICE,MARKER,info[SERVICE])
    managed_unit(units/OLD,OLD_MARKER,info[OLD])
    current=info[SERVICE]
    for name,row in info.items():
        if row.get('ActiveState') not in ('','inactive','failed','active'):
            raise ControlError('服务正在启动或停止，保留当前控制器并等待现场确认：'+name)
        if row.get('UnitFileState') in ('linked','linked-runtime','masked','masked-runtime'):
            raise ControlError('保留特殊链接或屏蔽开机策略，拒绝变更：'+name)
    mains={int(row.get('MainPID') or 0) for row in info.values()}-{0}
    for controller in _controllers():
        if controller.uid!=os.getuid() or not any(_descendant(controller.pid,main) for main in mains):
            raise ControlError('发现非托管控制器，保留原进程，拒绝启动第二个控制器')
    for name,row in info.items():
        result=run(['/usr/bin/systemctl','show',name,'--property=WorkingDirectory'],capture_output=True,text=True,check=False)
        row['WorkingDirectory']=next((s.split('=',1)[1] for s in (result.stdout or '').splitlines() if s.startswith('WorkingDirectory=')),'')
    return info


# 写服务/YAML及原开机策略的私有备份，附SHA；打印精确回滚命令，不覆盖外来运行参数。
def _snapshot(entry,home,project,unit,info,binary,agent,boot_policy,units):
    folder=home/'FC01_navigation';private_dir(folder)
    backups=folder/'service-backups';private_dir(backups)
    backup=backups/datetime.now().strftime('%Y%m%d_%H%M%S_%f');backup.mkdir(mode=0o700)
    path=units/SERVICE
    original=path.read_bytes() if path.is_file() else None
    if original is not None:(backup/SERVICE).write_bytes(original)
    old_path=units/OLD
    old_row=dict(info[OLD],present=old_path.is_file(),sha256=_digest(old_path) if old_path.is_file() else None)
    if old_row['present']:(backup/OLD).write_bytes(old_path.read_bytes())
    runtime=folder/'runtime.yaml'
    if runtime.is_symlink():raise ControlError('保留符号链接运行参数，拒绝覆盖')
    runtime_row={'present':runtime.is_file()}
    if runtime_row['present']:
        data=runtime.read_bytes();(backup/'runtime.yaml').write_bytes(data)
        runtime_row.update(sha256=hashlib.sha256(data).hexdigest(),mode=runtime.stat().st_mode&0o777)
    manifest=dict(service=SERVICE,entry=str(entry),project=str(project),original_present=original is not None,
        original_sha256=hashlib.sha256(original).hexdigest() if original is not None else None,
        unit_sha256=hashlib.sha256(unit.encode()).hexdigest(),original_unit_file_state=info[SERVICE].get('UnitFileState',''),
        original_active_state=info[SERVICE].get('ActiveState',''),original_working_directory=info[SERVICE].get('WorkingDirectory',''),
        boot_policy=boot_policy,controller=str(binary.resolve()),controller_sha256=_digest(binary),
        agent_pid=agent.pid,runtime=runtime_row,old_service=old_row)
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    _activation_state(entry,backup,'prepared')
    print('FC01_ROLLBACK_PATH='+str(backup),flush=True)
    print('FC01_ROLLBACK_COMMAND=bash '+shlex.quote(str(entry/'navigation_restore.sh'))+
          ' rollback --backup '+shlex.quote(str(backup)),flush=True)
    return backup,manifest


# 请求systemd自身优雅停止并持续检查Agent；超时只拒绝切换，不强杀仍在交权的控制器。
def _stop(entry,home,project,agent,run,*,timeout=25.,services=(SERVICE,)):
    """Request the unit's own graceful stop, never kill controller or Agent."""
    _ground(entry,home,project,agent)
    for name in services:run(['sudo','-n','/usr/bin/systemctl','stop','--no-block',name],check=True)
    deadline=time.monotonic()+timeout
    while True:
        retained_agent(agent)
        states=[unit_info(name,run) for name in services]
        if all(row.get('MainPID','0')=='0' and row.get('ActiveState') in ('inactive','failed','') for row in states) and not _controllers():return
        if time.monotonic()>=deadline:
            raise ControlError('旧控制器尚未退出；保持原服务文件与 Agent，不切换、不强制结束；等待现场安全处理')
        time.sleep(.2)


# 启动验收同时核对真实exe/cwd/SHA、唯一进程、boot/PID/版本/源配置和新鲜未解锁静止反馈。
def verify_running(entry,home,binary,agent,run=subprocess.run,*,proc=Path('/proc')):
    info=unit_info(SERVICE,run);main=int(info.get('MainPID') or 0)
    if info.get('ActiveState')!='active' or not main:raise ControlError('新版服务尚未处于 active')
    controllers=_controllers()
    # 必须只有一个原生控制发布者进程；不是“至少一个”就算启动成功。
    if len(controllers)!=1:raise ControlError('必须恰好存在一个 fc01_system_node，当前='+str(len(controllers)))
    controller=controllers[0];pid=controller.pid
    if controller.uid!=os.getuid() or not _descendant(pid,main,proc):raise ControlError('控制器进程不属于新版服务')
    process_dir=proc/str(pid)
    try:
        if (process_dir/'exe').resolve(strict=True)!=binary.resolve(strict=True):raise ControlError('实际控制器路径不属于新版')
        if _digest(process_dir/'exe')!=_digest(binary):raise ControlError('运行中控制器摘要不匹配')
        if (process_dir/'cwd').resolve(strict=True)!=entry.resolve():raise ControlError('控制器工作目录不属于新版')
        if (proc/str(main)/'cwd').resolve(strict=True)!=entry.resolve():raise ControlError('服务工作目录不属于新版')
        row=json.loads((home/'FC01_navigation/status.json').read_text())
        stamp=row.get('updated_unix');age=row.get('state_age_seconds')
        if (type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=time.time()-stamp<=3 or
            type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=2.5 or
            row.get('boot_id')!=(proc/'sys/kernel/random/boot_id').read_text().strip() or
            row.get('entry_version')!=ENTRY_VERSION or row.get('supervisor_pid')!=main or
            row.get('controller_pid')!=pid or row.get('state_controller_pid')!=pid or row.get('feedback_current') is not True):
            raise ControlError('新版状态反馈未匹配当前版本、进程或新鲜度')
        # 运行模式与源/实际YAML路径都须对应本entry，避免新服务加载旧参数。
        if (row.get('runtime_mode')!='flight-test' or
            row.get('config_source_path')!=str((entry/'native/config/flight_test.yaml').resolve()) or
            row.get('config_path')!=str((home/'FC01_navigation/runtime.yaml').resolve())):
            raise ControlError('新版运行模式或参数来源未匹配本版')
        state=row.get('state') or {}
        if state.get('vertical_feedback_velocity_policy')!='diagnostic_only':raise ControlError('新版垂向速度策略验收失败')
        if row.get('status') in ('FAULT','FAILED','STOPPING','STOPPED') or state.get('phase')=='SAFETY':
            raise ControlError('新版处于故障或停止状态，不能通过启动验收')
        if (state.get('armed') is not False or
            any(state.get(k) is not True for k in ('arming_state_known','status_fresh','land_fresh','landed','at_rest'))):
            raise ControlError('新版尚未确认新鲜未解锁接地静止状态')
    except (OSError,ValueError,TypeError) as exc:
        raise ControlError('无法验证新版进程或状态：'+str(exc)) from exc
    retained_agent(agent)
    return row


# 仅在全部控制器退出后恢复；备份SHA/marker/权限及当前外来修改都需核验，恢复后不自动启动。
def _restore_snapshot(home,backup,manifest,units,run):
    """Caller must establish that all controllers have exited."""
    if _controllers():raise ControlError('控制器仍存活，保留配置及回滚记录，拒绝恢复运行参数')
    path=units/SERVICE
    old=manifest.get('old_service')
    if old is not None:
        old_path=units/OLD
        current=_digest(old_path) if old_path.is_file() else None
        if old_path.is_symlink() or current!=old.get('sha256'):raise ControlError('旧待命服务文件已变化，保留它的开机策略')
        if old.get('present'):
            saved=backup/OLD
            if saved.is_symlink() or _digest(saved)!=old.get('sha256') or not saved.read_text().startswith(OLD_MARKER+'\n'):
                raise ControlError('旧待命服务备份无效，未恢复')
    runtime=manifest.get('runtime')
    if runtime is not None:
        live=home/'FC01_navigation/runtime.yaml'
        if live.is_symlink():raise ControlError('保留符号链接运行参数，拒绝覆盖')
        if runtime.get('present'):
            source=backup/'runtime.yaml'
            if source.is_symlink() or _digest(source)!=runtime.get('sha256'):raise ControlError('原运行参数备份摘要不匹配')
            mode=runtime.get('mode',0o600)
            if type(mode) is not int or mode&~0o777:raise ControlError('原运行参数权限记录无效')
    if manifest.get('original_present') is True:
        restored=backup/SERVICE
        if restored.is_symlink() or _digest(restored)!=manifest.get('original_sha256') or not restored.read_text().startswith(MARKER+'\n'):
            raise ControlError('原服务备份无效，拒绝恢复')
        run(['sudo','-n','/usr/bin/install','-m','0644',str(restored),str(path)],check=True)
    elif path.exists():
        run(['sudo','-n','/usr/bin/systemctl','disable',SERVICE],check=True)
        run(['sudo','-n','/usr/bin/rm','--',str(path)],check=True)
    run(['sudo','-n','/usr/bin/systemctl','daemon-reload'],check=True)
    state=manifest.get('original_unit_file_state')
    if state=='enabled':run(['sudo','-n','/usr/bin/systemctl','enable',SERVICE],check=True)
    elif state=='enabled-runtime':
        run(['sudo','-n','/usr/bin/systemctl','disable',SERVICE],check=True)
        run(['sudo','-n','/usr/bin/systemctl','enable','--runtime',SERVICE],check=True)
    elif state=='disabled' and manifest.get('original_present') is True:
        run(['sudo','-n','/usr/bin/systemctl','disable',SERVICE],check=True)
    if old is not None:
        old_state=old.get('UnitFileState')
        if old_state=='enabled':run(['sudo','-n','/usr/bin/systemctl','enable',OLD],check=True)
        elif old_state=='enabled-runtime':
            run(['sudo','-n','/usr/bin/systemctl','disable',OLD],check=True)
            run(['sudo','-n','/usr/bin/systemctl','enable','--runtime',OLD],check=True)
        elif old_state=='disabled':run(['sudo','-n','/usr/bin/systemctl','disable',OLD],check=True)
    if runtime is not None:
        if runtime.get('present'):
            staged=live.with_suffix('.yaml.restore-new');staged.write_bytes(source.read_bytes())
            staged.chmod(mode);os.replace(staged,live)
        else:live.unlink(missing_ok=True)


# 完整在线事务：验证构建→发现身份→认证/备份→新地面证据→停旧→复验→切换→启动→真实验收。
def activate(entry,home,project,*,boot_policy='preserve',units=UNIT_DIR,run=subprocess.run,verify_timeout=25.):
    """One transaction: authenticate, passively authorize, stop, switch and verify."""
    if boot_policy not in ('enable','preserve'):raise ControlError('boot policy must be enable or preserve')
    from navigation_build_evidence import verify_build
    verify_build(entry,project)
    binary=check_controller(entry=entry)
    if not binary.resolve().is_relative_to(entry.resolve()):raise ControlError('控制器不属于本版目录')
    info=_discover(units,run);agent=retained_agent()
    user=pwd.getpwuid(os.getuid()).pw_name
    unit=render_unit(user,home,entry,project,'flight-test',True)
    path=units/SERVICE
    # A repeat activation validates the actual running process and current IPC.
    # 重复activate也必须检查真实当前进程/反馈；不能仅unit文本相同就假定成功。
    if (path.is_file() and path.read_text()==unit and info[SERVICE].get('ActiveState')=='active' and boot_policy=='preserve' and
        info[OLD].get('MainPID','0')=='0' and info[OLD].get('UnitFileState') not in ('enabled','enabled-runtime')):
        verify_running(entry,home,binary,agent,run)
        state_path=entry/'ACTIVATION_STATE.json'
        if state_path.is_file():
            prior=json.loads(state_path.read_text());backup=Path(prior['backup'])
            allowed=(home/'FC01_navigation/service-backups').resolve()
            if (not backup.resolve().is_relative_to(allowed) or backup.resolve()==allowed or
                not (backup/'manifest.json').is_file() or prior.get('service')!=SERVICE or prior.get('entry')!=str(entry.resolve())):
                raise ControlError('已有启用状态的回滚记录无效，保留当前服务')
        else:backup,_=_snapshot(entry,home,project,unit,info,binary,agent,boot_policy,units)
        _activation_state(entry,backup,'active',already_active=True)
        print('FC01_NEW_SERVICE_ACTIVE_ALREADY_VERIFIED rollback='+str(backup),flush=True)
        return backup
    with tempfile.TemporaryDirectory(prefix='fc01_activate_') as temp:
        staged=Path(temp)/SERVICE;staged.write_text(unit)
        run(['/usr/bin/systemd-analyze','verify',str(staged)],check=True)
        run(['sudo','-v'],check=True)
        backup,manifest=_snapshot(entry,home,project,unit,info,binary,agent,boot_policy,units)
        # changed标识是否需回写备份，started标识是否需安全停新进程；两者独立避免错误清理。
        changed=False;started=False
        try:
            _ground(entry,home,project,agent)
            # A fresh authorization precedes any stop. No unit changes until exit.
            active=[name for name,row in info.items() if row.get('MainPID','0')!='0' or row.get('ActiveState')=='active']
            if active:
                _activation_state(entry,backup,'stopping_old')
                _stop(entry,home,project,agent,run,services=active)
            if info[OLD].get('UnitFileState') in ('enabled','enabled-runtime'):
                if (_digest(units/OLD) if (units/OLD).is_file() else None)!=manifest['old_service'].get('sha256'):
                    raise ControlError('旧待命服务文件已变化，拒绝修改其开机策略')
                changed=True
                command=['sudo','-n','/usr/bin/systemctl','disable']
                if info[OLD].get('UnitFileState')=='enabled-runtime':command.append('--runtime')
                run([*command,OLD],check=True)
            # 任何安装/运行参数变更前都确认旧控制器已经退出，不能靠stop命令返回就假定完成。
            stopped_services(units,run)
            checked_binary,checked_agent=existing_agent(entry,home,project)
            if checked_binary!=binary or checked_agent!=agent:raise ControlError('切换前控制器或 Agent 已变化')
            verify_build(entry,project)
            _ground(entry,home,project,agent)
            managed_unit(path,MARKER,unit_info(SERVICE,run))
            current=_digest(path) if path.is_file() else None
            if current!=manifest.get('original_sha256'):raise ControlError('旧服务文件已被其他程序修改，拒绝覆盖')
            changed=True
            run(['sudo','-n','/usr/bin/install','-m','0644',str(staged),str(path)],check=True)
            run(['sudo','-n','/usr/bin/systemctl','daemon-reload'],check=True)
            if boot_policy=='enable':run(['sudo','-n','/usr/bin/systemctl','enable',SERVICE],check=True)
            _activation_state(entry,backup,'configured')
            # reload/enable may outlive a 2.5s permit: always reacquire before start.
            _ground(entry,home,project,agent)
            stopped_services(units,run)
            started=True
            run(['sudo','-n','/usr/bin/systemctl','start',SERVICE],check=True)
            _activation_state(entry,backup,'verifying')
            deadline=time.monotonic()+verify_timeout
            while True:
                try:
                    row=verify_running(entry,home,binary,agent,run);break
                except (ControlError,OSError,ValueError) as exc:
                    if time.monotonic()>=deadline:raise ControlError('新版启动验收失败：'+str(exc)) from exc
                    time.sleep(.2)
            _activation_state(entry,backup,'active',controller_pid=row['controller_pid'],agent_pid=agent.pid)
            print('FC01_NEW_SERVICE_ACTIVE version='+ENTRY_VERSION+' vertical_feedback_velocity_policy=diagnostic_only',flush=True)
            print('FC01_EXISTING_AGENT_RETAINED pid='+str(agent.pid)+' rollback='+str(backup),flush=True)
            return backup
        # 失败先安全停新控制器，再在无外来变更前提下还原；恢复自身失败则保留人工恢复证据。
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
            try:
                if started:_stop(entry,home,project,agent,run)
                if changed:
                    stopped_services(units,run)
                    if path.is_file() and _digest(path) not in (manifest['unit_sha256'],manifest.get('original_sha256')):
                        raise ControlError('当前服务文件已被其他程序修改，保留它')
                    _restore_snapshot(home,backup,manifest,units,run)
                _activation_state(entry,backup,'failed_restored' if changed else 'failed_unchanged',error=str(exc))
            except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as recovery:
                _activation_state(entry,backup,'failed_requires_manual_recovery',error=str(exc),recovery_error=str(recovery))
                raise ControlError(str(exc)+'；安全恢复未完成：'+str(recovery)+'；回滚记录='+str(backup)) from exc
            raise ControlError(str(exc)+'；原配置已保留/恢复，控制器不会自动重启；回滚记录='+str(backup)) from exc


# 只检查，不隐式stop/kill；旧开机入口仍有效或有覆盖配置时拒绝第二个入口。
def stopped_services(units, run):
    """No stop/kill is implicit: stop any old starter before configuring."""
    info = {name: unit_info(name, run) for name in (SERVICE, OLD)}
    for name, row in info.items():
        if row.get('MainPID', '0') != '0' or row.get('ActiveState') not in ('inactive', 'failed', ''):
            raise ControlError('服务仍在运行，先在接地上锁后人工停止：'+name)
        if row.get('DropInPaths'):
            raise ControlError('存在服务覆盖配置，保留它并拒绝变更：'+name)
    if info[OLD].get('UnitFileState') in ('enabled', 'enabled-runtime', 'linked', 'linked-runtime'):
        raise ControlError('旧待命服务仍有开机入口；接地上锁后人工确认并执行：sudo systemctl disable '+OLD)
    if any(Path(p.argv[0]).name == 'fc01_system_node' for p in process_list()):
        raise ControlError('已有控制器运行；保留它，拒绝配置第二个入口')
    return info


# 预检当前entry内二进制和已存在Agent；SDK只提供消息依赖，不能成为旧控制器来源。
def existing_agent(entry, home, project):
    binary = check_controller(entry=entry)
    if not binary.resolve().is_relative_to(entry.resolve()):
        raise ControlError('新版控制器指向其他目录，拒绝启动旧版二进制')
    _, agent = preflight(project, home/'fc01_agent_build_offline/MicroXRCEAgent', '/dev/ttyS2',
                         controller=binary, config_path=entry/'native/config/flight_test.yaml')
    if agent is None:
        raise ControlError('未找到可复用的既有 Agent；本工具不启动或停止 Agent')
    return binary, agent


# 只安装/按策略enable服务定义，不立即start；密码后获取新地面证据且保存可回滚原unit。
def configure(entry, home, project, *, boot_policy='enable', units=UNIT_DIR, run=subprocess.run):
    if boot_policy not in ('enable', 'preserve'):
        raise ControlError('boot policy must be enable or preserve')
    info = stopped_services(units, run)
    path = units/SERVICE
    managed_unit(path, MARKER, info[SERVICE])
    binary, agent = existing_agent(entry, home, project)
    folder = home/'FC01_navigation'
    private_dir(folder)
    config = folder/'restore-flight-test-runtime.yaml'
    boot(entry, home, project, 'prepare-config', '--mode', 'flight-test',
         '--confirm-flight-test', '--file', config)
    boot(entry, home, project, 'check-config', '--mode', 'flight-test',
         '--confirm-flight-test', '--file', config, '--prefix', entry/'install')
    user = pwd.getpwuid(os.getuid()).pw_name
    # Service availability is persistent; task/RC authorization is reacquired
    # from fresh native feedback after every process start. Enabling the unit
    # neither starts the service now nor authorizes an arm/takeoff action.
    unit = render_unit(user, home, entry, project, 'flight-test', True)
    with tempfile.TemporaryDirectory(prefix='fc01_restore_unit_') as temp:
        staged = Path(temp)/SERVICE
        staged.write_text(unit)
        run(['/usr/bin/systemd-analyze', 'verify', str(staged)], check=True)
        run(['sudo', '-v'], check=True)
        # Password entry must precede new, passive ground evidence.
        boot(entry, home, project, 'probe', '--file', folder/'probe.json')
        from navigation_install import ground_permit_fresh
        if not ground_permit_fresh(home):
            raise ControlError('缺少新鲜未解锁接地证据，未变更服务')
        stopped_services(units, run)
        checked_binary, checked_agent = existing_agent(entry, home, project)
        if checked_binary != binary or checked_agent != agent:
            raise ControlError('校验后控制器或 Agent 已变化，未变更服务')
        managed_unit(path, MARKER, unit_info(SERVICE, run))
        backups = folder/'service-backups'
        private_dir(backups)
        backup = backups/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        backup.mkdir(mode=0o700)
        present = path.is_file()
        original = path.read_bytes() if present else None
        if present:
            (backup/SERVICE).write_bytes(original)
        manifest={
            'service': SERVICE, 'original_present': present, 'entry': str(entry),
            'project': str(project), 'unit_sha256': hashlib.sha256(unit.encode()).hexdigest(),
            'original_unit_file_state': info[SERVICE].get('UnitFileState', ''),
            'boot_policy': boot_policy,
            'original_sha256': hashlib.sha256(original).hexdigest() if present else None,
            'controller': str(binary.resolve()),
            'controller_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'agent_pid': agent.pid,
        }
        (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        if not ground_permit_fresh(home):
            raise ControlError('接地证据已过期，未变更服务')
        # Never silently remove existing persistent service enablement.
        # Does not start, stop, kill, or replace any binary.
        try:
            run(['sudo', '-n', '/usr/bin/install', '-m', '0644', str(staged), str(path)], check=True)
            run(['sudo', '-n', '/usr/bin/systemctl', 'daemon-reload'], check=True)
            if boot_policy == 'enable':
                run(['sudo', '-n', '/usr/bin/systemctl', 'enable', SERVICE], check=True)
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
            try:
                stopped_services(units,run)
                current=_digest(path) if path.is_file() else None
                if current not in (manifest['unit_sha256'],manifest['original_sha256']):
                    raise ControlError('当前服务已被其他程序修改，拒绝覆盖')
                _restore_snapshot(home,backup,manifest,units,run)
            except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as recovery:
                raise ControlError(str(exc)+'；安全恢复未完成：'+str(recovery)+'；rollback='+str(backup)) from exc
            raise ControlError(str(exc)+'；原服务配置已恢复且未启动；rollback='+str(backup)) from exc
    print('FC01_NEW_SERVICE_CONFIGURED controller='+str(binary.resolve()), flush=True)
    print('FC01_EXISTING_AGENT_RETAINED pid='+str(agent.pid), flush=True)
    effective_state = 'enabled' if boot_policy == 'enable' else info[SERVICE].get('UnitFileState', '')
    print('FC01_SERVICE_NOT_STARTED boot_enabled='+effective_state+' boot_policy='+boot_policy+
          ' original_enabled_state='+info[SERVICE].get('UnitFileState', '')+
          ' rollback='+str(backup), flush=True)
    return backup


# 备份必须属于当前用户service-backups；当前unit未被他人修改且地面许可新鲜才恢复，保持停止。
def rollback(entry, home, project, backup, *, units=UNIT_DIR, run=subprocess.run, stop_current=False):
    backup = backup.expanduser().resolve()
    allowed = (home/'FC01_navigation/service-backups').resolve()
    if not backup.is_relative_to(allowed) or backup == allowed:
        raise ControlError('回滚记录必须位于当前用户 service-backups 目录')
    manifest = json.loads((backup/'manifest.json').read_text())
    if manifest.get('service') != SERVICE:
        raise ControlError('回滚记录服务名不匹配')
    path = units/SERVICE
    managed_unit(path, MARKER, unit_info(SERVICE, run))
    current=_digest(path) if path.is_file() else None
    if current not in (manifest.get('unit_sha256'),manifest.get('original_sha256')):
        raise ControlError('当前服务已被修改，保留它；拒绝覆盖其他变更')
    restored = backup/SERVICE
    if manifest.get('original_present') is True:
        if (restored.is_symlink() or not restored.is_file() or
                not restored.read_text().startswith(MARKER+'\n') or
                hashlib.sha256(restored.read_bytes()).hexdigest() != manifest.get('original_sha256')):
            raise ControlError('原服务备份无效，未回滚')
    run(['sudo', '-v'], check=True)
    if stop_current:
        info=_discover(units,run);agent=retained_agent()
        active=[name for name,row in info.items() if row.get('MainPID','0')!='0' or row.get('ActiveState')=='active']
        if active:
            _stop(entry,home,project,agent,run,services=active)
        _ground(entry,home,project,agent)
    else:
        stopped_services(units,run)
        boot(entry, home, project, 'probe', '--file', home/'FC01_navigation/probe.json')
    from navigation_install import ground_permit_fresh
    if not ground_permit_fresh(home):
        raise ControlError('缺少新鲜未解锁接地证据，未回滚')
    if stop_current:
        info=_discover(units,run)
        if any(row.get('MainPID','0')!='0' or row.get('ActiveState') not in ('inactive','failed','') for row in info.values()) or _controllers():
            raise ControlError('控制器尚未退出，拒绝回滚配置')
    else:stopped_services(units, run)
    current=_digest(path) if path.is_file() else None
    if current not in (manifest.get('unit_sha256'),manifest.get('original_sha256')):
        raise ControlError('安全检查后服务文件已变化，保留它，拒绝回滚覆盖')
    _restore_snapshot(home,backup,manifest,units,run)
    if stop_current:_activation_state(entry,backup,'rolled_back_not_started')
    print('FC01_SERVICE_ROLLED_BACK_NOT_STARTED; 原服务保持停止，需现场人工选择是否启动', flush=True)


# 第一次中断转为可恢复异常，随后忽略重复信号，避免父进程再转SIGTERM打断安全恢复。
def _install_interrupt_handlers():
    interrupted_once=False
    signals=(signal.SIGTERM,signal.SIGHUP,signal.SIGINT)
    # 信号处理只触发一次受控异常；不直接杀节点或清理共享文件。
    def interrupted(signum,frame):
        nonlocal interrupted_once
        if interrupted_once:return
        interrupted_once=True
        # Ctrl-C reaches this process directly and the parent may forward
        # SIGTERM as well. A second signal must not interrupt safe recovery.
        for item in signals:signal.signal(item,signal.SIG_IGN)
        raise ControlError('收到中断信号，执行安全收尾；不会自动重启控制器')
    for signum in signals:signal.signal(signum,interrupted)
    return interrupted


# 普通用户入口、显式确认、完整安装锁；控制器未安全退出时只返回保留/停止状态。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('configure', 'activate', 'rollback'))
    parser.add_argument('--project-root', type=Path, required=True,
                        help='现有 ROS/px4_msgs SDK 根目录；不作为控制器或参数来源')
    parser.add_argument('--confirm-flight-test', action='store_true')
    parser.add_argument('--boot-policy', choices=('enable', 'preserve'), default='preserve',
                        help='默认保留原开机策略；显式 enable 才启用开机入口')
    parser.add_argument('--backup', type=Path)
    args = parser.parse_args(argv)
    if os.getuid() == 0:
        parser.error('请以 orangepi 普通用户运行；只在写服务时使用 sudo')
    if args.action in ('configure','activate') and not args.confirm_flight_test:
        parser.error(args.action+' 必须显式提供 --confirm-flight-test')
    if args.action == 'rollback' and args.backup is None:
        parser.error('rollback requires --backup')
    os.umask(0o077)
    _install_interrupt_handlers()
    home = Path(pwd.getpwuid(os.getuid()).pw_dir)
    project = args.project_root.expanduser().resolve()
    # navigation_restore.sh supplies the reviewed ROS/px4_msgs environment.
    os.environ['FC01_PROJECT_ROOT'] = str(project)
    private_dir(home/'.cache/fc01_navigation')
    try:
        with open(home/'.cache/fc01_navigation/install.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if args.action == 'configure':
                configure(ENTRY, home, project, boot_policy=args.boot_policy)
            elif args.action == 'activate':
                activate(ENTRY, home, project, boot_policy=args.boot_policy)
            else:
                rollback(ENTRY, home, project, args.backup, stop_current=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print('FC01_SERVICE_UNCHANGED_OR_STOPPED: '+str(exc), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
