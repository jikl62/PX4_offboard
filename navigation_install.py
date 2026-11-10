#!/usr/bin/env python3
"""Build an isolated native overlay, validate, observe ground, then deploy."""
from __future__ import annotations
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import pwd
import re
import shutil
import subprocess
import tempfile
import time

from autostart import MARKER as OLD_MARKER,quote
from launcher import PROJECT,private_dir,process_list

SERVICE='fc01-navigation.service'
OLD='fc01-standby.service'
MARKER='# FC01_RC_NAVIGATION_MANUAL_ARM_20261028_V1'
DESKTOP_MARKER='# FC01_NAVIGATION_READ_ONLY_VIEW_20261028_V1'
MODES=('diagnostic','flight-test')

def render_unit(user,home,entry,project,mode='diagnostic',confirm=False):
    if mode not in MODES:raise ValueError('运行模式必须为 diagnostic 或 flight-test')
    if mode=='flight-test' and not confirm:raise ValueError('Flight Test 服务必须显式确认')
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*\$?',user):raise ValueError('不支持此用户名')
    quote(entry)
    if not entry.is_absolute():raise ValueError('服务目录必须是绝对路径')
    return f'''{MARKER}
[Unit]
Description=FC01 manual RC navigation controller
After=local-fs.target
StartLimitIntervalSec=120s
StartLimitBurst=5
[Service]
Type=simple
User={user}
WorkingDirectory={str(entry).replace('%','%%')}
Environment={quote('HOME='+str(home))}
Environment={quote('FC01_PROJECT_ROOT='+str(project))}
Environment=ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1
ExecStart=/bin/bash {quote(entry/'navigation_boot.sh',executable=True)} run --mode {mode}{' --confirm-flight-test' if mode=='flight-test' else ''}
StandardInput=null
StandardOutput=journal
StandardError=journal
SyslogIdentifier=fc01-navigation
UMask=0077
NoNewPrivileges=true
Restart=on-failure
RestartSec=30s
KillMode=process
KillSignal=SIGTERM
SendSIGKILL=no
TimeoutStopSec=infinity
[Install]
WantedBy=multi-user.target
'''

def render_desktop(entry):
    quote(entry)
    # Exec has its own quoting layer; the key-file layer doubles backslashes.
    value=str(entry/'navigation.sh').replace('%','%%')
    value=''.join('\\'+c if c in '\\"`$' else c for c in value)
    value=value.replace('\\','\\\\')
    return f'''{DESKTOP_MARKER}
[Desktop Entry]
Type=Application
Name=FC01 导航控制状态
Comment=只读状态窗口；关闭窗口不停止控制
Exec=/bin/bash "{value}" monitor
Terminal=true
StartupNotify=false
X-GNOME-Autostart-enabled=true
'''

def source_key(entry,project):
    h=hashlib.sha256()
    for root in (entry/'native/src',project/'install_r632/px4_msgs/include'):
        for p in sorted(root.rglob('*')):
            if p.is_file():h.update(str(p.relative_to(root)).encode());h.update(p.read_bytes())
    h.update(os.uname().machine.encode())
    compiler=shutil.which('g++')
    if compiler:
        st=Path(compiler).stat()
        h.update(str((str(Path(compiler).resolve()),st.st_size,st.st_mtime_ns)).encode())
    return h.hexdigest()[:24]

def build_complete(prefix):
    binaries=[prefix/'lib/jikl_fc01_system'/name for name in ('fc01_system_node','fc01_config_check')]
    try:
        saved=json.loads((prefix.parent/'build-complete.json').read_text())
        return saved=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries}
    except (OSError,ValueError):return False

def staging_path(entry,key):
    for attempt in range(1000):
        stage=entry/'.build'/(key+(f'.retry{attempt}' if attempt else ''))
        if not (stage/'install').exists() or build_complete(stage/'install'):return stage
    raise RuntimeError('过多未完成构建；未变更现有服务')

def build_native(entry,prefix,build,run=subprocess.run):
    if any(shutil.which(x) is None for x in ('cmake','g++')):
        raise RuntimeError('缺少已安装的 cmake 或 g++；未下载依赖，未变更服务')
    if build_complete(prefix):return
    # Never write a prefix that may be in use if its completion record changed.
    if prefix.exists():raise RuntimeError('已有构建目录不完整或被修改，保留它；报告中记录路径：'+str(prefix))
    run(['cmake','-S',str(entry/'native/src/jikl_fc01_system'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(prefix)],check=True)
    run(['cmake','--build',str(build),'--parallel','1'],check=True)
    run(['cmake','--install',str(build)],check=True)
    binaries=[prefix/'lib/jikl_fc01_system'/name for name in ('fc01_system_node','fc01_config_check')]
    if all(p.is_file() for p in binaries):
        (prefix.parent/'build-complete.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries}))

def boot(entry,home,project,action,*args):
    from standby import rotating_logger
    folder=home/'FC01_navigation';private_dir(folder)
    log=rotating_logger(folder/'install.log',1024*1024)
    try:
        command=['/bin/bash',str(entry/'navigation_boot.sh'),action,*map(str,args)]
        log.info('运行 %s',command)
        with subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                              env={**os.environ,'HOME':str(home),'FC01_PROJECT_ROOT':str(project)}) as child:
            while True:
                block=os.read(child.stdout.fileno(),4096)
                if not block:break
                log.info('%s',block.decode(errors='replace').rstrip())
            if child.wait()!=0:raise RuntimeError(action+' 失败；详见自动报告 install.log')
    finally:
        for handler in log.handlers:handler.close()

def prepare(entry,home,project,mode='diagnostic',confirm=False):
    if mode not in MODES:raise ValueError('运行模式必须为 diagnostic 或 flight-test')
    if mode=='flight-test' and not confirm:raise ValueError('Flight Test 必须显式确认')
    folder=home/'FC01_navigation';private_dir(folder)
    config=folder/'install-runtime.yaml'
    boot(entry,home,project,'prepare-config','--mode',mode,*(['--confirm-flight-test'] if mode=='flight-test' else []),'--file',config)
    key=source_key(entry,project);stage=staging_path(entry,key)
    print('正在离线编译 ARM64 原生控制器；单任务构建，首次需要等待。',flush=True)
    boot(entry,home,project,'build','--prefix',stage/'install','--build-dir',stage/'build')
    boot(entry,home,project,'check-config','--mode',mode,*(['--confirm-flight-test'] if mode=='flight-test' else []),'--file',config,'--prefix',stage/'install')
    print('编译和实际 YAML 检查通过；正在被动确认未解锁、接地、静止。',flush=True)
    boot(entry,home,project,'probe','--mode',mode,*(['--confirm-flight-test'] if mode=='flight-test' else []),'--file',folder/'probe.json')
    return stage/'install'

def unit_info(name,run):
    result=run(['/usr/bin/systemctl','show',name,'--property=FragmentPath,DropInPaths,MainPID,ActiveState,UnitFileState'],capture_output=True,text=True,check=False)
    return dict(line.split('=',1) for line in (result.stdout or '').splitlines() if '=' in line)

def managed_unit(path,marker,info):
    if path.is_symlink():raise RuntimeError('保留符号链接服务：'+str(path))
    if path.exists() and not path.read_text().startswith(marker+'\n'):raise RuntimeError('保留不属于本包的服务：'+str(path))
    if info.get('DropInPaths') or info.get('FragmentPath') not in ('',str(path),None):raise RuntimeError('服务存在其他来源或覆盖配置，未变更：'+str(path))
    if info.get('MainPID','0')!='0' and not path.exists():raise RuntimeError('服务进程来源不明，未变更')

def check_controllers(processes,allowed_pids):
    for process in processes:
        if not any(Path(arg).name=='fc01_system_node' for arg in process.argv[:2]):continue
        pid=process.pid;seen=set()
        while pid>1 and pid not in seen and pid not in allowed_pids:
            seen.add(pid)
            try:
                fields=Path('/proc',str(pid),'status').read_text().splitlines()
                pid=int(next(x.split()[1] for x in fields if x.startswith('PPid:')))
            except (OSError,ValueError,StopIteration):break
        if pid not in allowed_pids:raise RuntimeError('发现非托管控制器，保留其进程，拒绝启动第二个控制器')

def desktop_paths(home):
    return [home/'Desktop/FC01_Navigation_Status.desktop',home/'.config/autostart/FC01_Navigation_Status.desktop']

def check_desktop(home):
    for p in desktop_paths(home):
        if p.is_symlink() or (p.exists() and not p.read_text().startswith(DESKTOP_MARKER+'\n')):
            raise RuntimeError('保留不属于本包的桌面入口：'+str(p))

def ground_permit_fresh(home):
    try:
        row=json.loads((home/'FC01_navigation/probe.json').read_text())
        stamp=row.get('updated_monotonic')
        return (row.get('ok') is True and row.get('armed_seen') is False and
                row.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and
                type(stamp) in (int,float) and math.isfinite(stamp) and 0<=time.monotonic()-stamp<=2.5)
    except (OSError,ValueError,AttributeError):return False

def authenticate_admin(run):
    # Password entry may take arbitrarily long. No ground authorization
    # obtained before this prompt is used to mutate a service afterward.
    run(['sudo','-v'],check=True)

def ensure_ground(entry,home,project,force=False):
    if force or not ground_permit_fresh(home):
        boot(entry,home,project,'probe','--file',home/'FC01_navigation/probe.json')
    if not ground_permit_fresh(home):raise RuntimeError('地面授权已过期，未继续变更服务')

def deploy(entry,home,project,prefix,*,user=None,units=Path('/etc/systemd/system'),run=subprocess.run,processes=None,mode='diagnostic',confirm=False):
    old,new=units/OLD,units/SERVICE
    info={name:unit_info(name,run) for name in (OLD,SERVICE)}
    managed_unit(old,OLD_MARKER,info[OLD]);managed_unit(new,MARKER,info[SERVICE]);check_desktop(home)
    check_controllers(process_list() if processes is None else processes,{int(x.get('MainPID') or 0) for x in info.values()}-{0})
    link=entry/'install'
    pending=entry/'.install-link-new'
    if pending.exists() or pending.is_symlink():raise RuntimeError('待发布链接已存在，保留它')
    if link.exists() and not link.is_symlink():raise RuntimeError('保留非托管 install 路径')
    if link.is_symlink() and (link.resolve().parent.parent!=entry/'.build'):
        raise RuntimeError('保留指向其他项目的 install 链接')
    unit=render_unit(user or pwd.getpwuid(os.getuid()).pw_name,home,entry,project,mode,confirm)
    prepared=home/'FC01_navigation/install-runtime.yaml';live=home/'FC01_navigation/runtime.yaml'
    config_changed=prepared.is_file() and (not live.is_file() or prepared.read_bytes()!=live.read_bytes())
    changed=not new.exists() or new.read_text()!=unit or not link.is_symlink() or link.resolve()!=prefix.resolve() or config_changed
    with tempfile.TemporaryDirectory(prefix='fc01_navigation_unit_') as folder:
        temp=Path(folder)/SERVICE;temp.write_text(unit)
        run(['/usr/bin/systemd-analyze','verify',str(temp)],check=True)
        # Every refusal above precedes sudo/service mutations.
        authenticate_admin(run)
        ensure_ground(entry,home,project,force=True)
        if old.exists():
            ensure_ground(entry,home,project)
            run(['sudo','-n','/usr/bin/systemctl','disable','--now',OLD],check=True)
        if changed and new.exists():
            ensure_ground(entry,home,project)
            run(['sudo','-n','/usr/bin/systemctl','stop',SERVICE],check=True)
        if changed:
            ensure_ground(entry,home,project)
            pending.symlink_to(prefix);os.replace(pending,link)
            run(['sudo','-n','/usr/bin/install','-m','0644',str(temp),str(new)],check=True)
            run(['sudo','-n','/usr/bin/systemctl','daemon-reload'],check=True)
        for p in desktop_paths(home):
            p.parent.mkdir(mode=0o700,parents=True,exist_ok=True);p.write_text(render_desktop(entry));p.chmod(0o700)
        ensure_ground(entry,home,project)
        run(['sudo','-n','/usr/bin/systemctl','enable','--now',SERVICE],check=True)
    print('导航服务已启用；当前终端显示状态，桌面登录后也会显示只读状态窗口。',flush=True)

def install(entry,home,mode='diagnostic',confirm=False):
    if os.getuid()==0:raise RuntimeError('请用 orangepi 普通用户运行；安装服务时会请求 sudo')
    project=Path(os.environ.get('FC01_PROJECT_ROOT',str(home/PROJECT))).expanduser().resolve()
    lockdir=home/'.cache/fc01_navigation';private_dir(lockdir)
    with open(lockdir/'install.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        prefix=prepare(entry,home,project,mode,confirm);deploy(entry,home,project,prefix,mode=mode,confirm=confirm)

def rollback_services(entry,home,project,units=Path('/etc/systemd/system'),run=subprocess.run):
    for name,marker in ((OLD,OLD_MARKER),(SERVICE,MARKER)):
        p=units/name;managed_unit(p,marker,unit_info(name,run))
        if not p.exists():raise RuntimeError('没有可验证的托管服务，未回滚：'+name)
    authenticate_admin(run)
    ensure_ground(entry,home,project,force=True)
    run(['sudo','-n','/usr/bin/systemctl','disable','--now',SERVICE],check=True)
    ensure_ground(entry,home,project)
    run(['sudo','-n','/usr/bin/systemctl','enable','--now',OLD],check=True)

def rollback(entry,home):
    project=Path(os.environ.get('FC01_PROJECT_ROOT',str(home/PROJECT))).expanduser().resolve()
    lockdir=home/'.cache/fc01_navigation';private_dir(lockdir)
    with open(lockdir/'install.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        boot(entry,home,project,'probe','--file',home/'FC01_navigation/probe.json')
        rollback_services(entry,home,project)
    print('已回到原关闭动作门待命服务；导航自动起飞已停用。',flush=True)
