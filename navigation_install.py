#!/usr/bin/env python3
# 导航旧安装管理链：隔离构建/配置检查/地面探针后才切换托管服务，默认诊断模式。
# 不要把此source_key复用合同与更严格ARM64_BUILD_EVIDENCE混同；运行态版本不得原地覆盖。
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

from autostart import MARKER as OLD_MARKER, quote
from launcher import PROJECT, private_dir, process_list
from navigation_build_evidence import BuildEvidenceError, tree_fingerprint

SERVICE = 'fc01-navigation.service'
OLD = 'fc01-standby.service'
MARKER = '# FC01_RC_NAVIGATION_MANUAL_ARM_20261028_V1'
DESKTOP_MARKER = '# FC01_NAVIGATION_READ_ONLY_VIEW_20261028_V1'
MODES = ('diagnostic', 'flight-test')


# 模式白名单与flight-test显式确认先校验；生成的服务使用监督器安全退出，不向子控制器强杀。
def render_unit(user, home, entry, project, mode='diagnostic', confirm=False):
    # 输入 user 为服务账号，home/entry/project 为明确路径；返回 unit 文本，不写文件或调用 systemd。
    # confirm 只允许生成 flight-test 文本，不替代后续地面证据；非法模式、用户名或路径直接抛错。
    if mode not in MODES:
        raise ValueError('运行模式必须为 diagnostic 或 flight-test')
    if mode == 'flight-test' and not confirm:
        raise ValueError('Flight Test 服务必须显式确认')
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*\$?', user):
        raise ValueError('不支持此用户名')
    quote(entry)
    if not entry.is_absolute():
        raise ValueError('服务目录必须是绝对路径')
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


# 桌面入口只是只读monitor；同时处理desktop键文件与Exec两层转义，关闭窗口不停止控制。
def render_desktop(entry):
    # 返回两层转义后的 desktop 文本；只包含 monitor 入口，不写入桌面目录。
    # quote 先拒绝危险路径字符；百分号还须按 desktop 字段规则转义，不是 shell 命令拼接。
    quote(entry)
    # Exec has its own quoting layer; the key-file layer doubles backslashes.
    value = str(entry / 'navigation.sh').replace('%', '%%')
    value = ''.join('\\' + c if c in '\\"`$' else c for c in value)
    value = value.replace('\\', '\\\\')
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


# 由真实源码/工具/消息/架构与编译器身份选择独立构建目录，不授予激活或飞行权限。
def source_key(entry, project):
    # 构建键决定使用哪个独立目录；相同输入复用，不覆盖可能正在运行的旧版本。
    # 返回24位十六进制目录键；只读源码/消息头和本机工具属性，文件读取错误原样传播。
    # 这是旧安装器的复用线索，不含正式依赖树全部证据，也不证明主机一定是ARM64。
    h = hashlib.sha256()
    # CMake 还会安装 native/tools 中的 Python 工具；它们变化也必须选择新构建目录。
    for root in (
        entry / 'native/src',
        entry / 'native/tools',
        project / 'install_r632/px4_msgs/include',
    ):
        # 固定遍历顺序，并同时摘要相对路径和内容，避免目录枚举顺序影响构建键。
        for p in sorted(root.rglob('*')):
            if p.is_file():
                h.update(str(p.relative_to(root)).encode())
                h.update(p.read_bytes())
    # 不同 CPU 架构的构建结果不能共用同一个候选目录。
    h.update(os.uname().machine.encode())
    compiler = shutil.which('g++')
    if compiler:
        # 保留原编译器身份策略：路径、大小和修改时间；读取产生的访问时间不触发重建。
        st = Path(compiler).stat()
        h.update(str((str(Path(compiler).resolve()), st.st_size, st.st_mtime_ns)).encode())
    # 保留既有 24 位十六进制目录键约定；这不是完整的 ARM64 构建验收证据。
    return h.hexdigest()[:24]


# 两个程序均存在且schema2整棵安装树指纹匹配才准复用；旧二进制-only记录一次失效。
def build_complete(prefix):
    # 控制器与配置检查器都是必需输出；仅有其中之一不能证明安装完成。
    # 只读返回 bool；证据不可读、JSON不合法或树摘要失败时按“不完整”处理。
    # 不完整不等于允许删旧产物；调用者必须另选目录，避免破坏仍在运行的版本。
    binaries = [
        prefix / 'lib/jikl_fc01_system' / name for name in ('fc01_system_node', 'fc01_config_check')
    ]
    try:
        saved = json.loads((prefix.parent / 'build-complete.json').read_text())
        # 两个 ELF 未变不代表消息库、Python 模块和工具未变；旧版二进制清单不再复用。
        return (
            saved.get('schema') == 2
            and all(p.is_file() for p in binaries)
            and saved.get('installed_files') == tree_fingerprint(prefix)
        )
    except (OSError, ValueError, AttributeError, BuildEvidenceError):
        # 证据缺失、格式错误或文件树无法确认时，保留旧目录并按未完成处理。
        return False


# 有限搜索未使用/完整目录；不完整旧安装保留并另选retry，避免覆盖运行服务使用的路径。
def staging_path(entry, key):
    # 尝试数量沿用原上限，避免大量残留目录导致无界搜索。
    # entry/key 决定候选路径，返回 Path 而不创建目录；优先复用首个可用项。
    # 缺 install 的目录可再次准备，已存在但证据不完整的 install 必须保留。
    for attempt in range(1000):
        stage = entry / '.build' / (key + (f'.retry{attempt}' if attempt else ''))
        # 尚无安装前缀可用于新构建，完整安装可复用；不完整安装只能保留并另选目录。
        if not (stage / 'install').exists() or build_complete(stage / 'install'):
            return stage
    # 搜索耗尽时停止准备，不切换服务或覆盖已有安装。
    raise RuntimeError('过多未完成构建；未变更现有服务')


# 只向独立prefix构建/安装，成功记录完整树；不启动服务或Agent。
def build_native(entry, prefix, build, run=subprocess.run):
    # 仅使用本机已安装的工具，不在部署过程中下载依赖。
    # prefix/build 是隔离安装与编译路径；run 可注入替身，默认会执行真实CMake。
    # 正常完成无返回值；子命令失败直接传播，部分产物保留，不自行切换服务或删除旧前缀。
    if any(shutil.which(x) is None for x in ('cmake', 'g++')):
        raise RuntimeError('缺少已安装的 cmake 或 g++；未下载依赖，未变更服务')
    if build_complete(prefix):
        # 完整安装树仍匹配证据时，避免重复编译及写入。
        return
    if prefix.exists():
        # 旧前缀可能仍被运行中的服务使用；证据失效也不能直接覆盖它。
        raise RuntimeError('已有构建目录不完整或被修改，保留它；报告中记录路径：' + str(prefix))
    # 先配置隔离的构建目录与安装前缀，配置失败立即向调用方报告。
    run(
        [
            'cmake',
            '-S',
            str(entry / 'native/src/jikl_fc01_system'),
            '-B',
            str(build),
            '-DCMAKE_BUILD_TYPE=Release',
            '-DCMAKE_INSTALL_PREFIX=' + str(prefix),
        ],
        check=True,
    )
    # 单任务编译沿用现有资源策略；本函数不启动控制器或 Agent。
    run(['cmake', '--build', str(build), '--parallel', '1'], check=True)
    # 仅安装到本次独立前缀，服务切换仍由后续单独流程负责。
    run(['cmake', '--install', str(build)], check=True)
    binaries = [
        prefix / 'lib/jikl_fc01_system' / name for name in ('fc01_system_node', 'fc01_config_check')
    ]
    if all(p.is_file() for p in binaries):
        # 只有两个必需程序均已安装才记录成功；整棵安装树摘要用于之后的复用校验。
        (prefix.parent / 'build-complete.json').write_text(
            json.dumps({'schema': 2, 'installed_files': tree_fingerprint(prefix)})
        )


# 在指定HOME/SDK环境执行同包子命令，完整输出写安装日志，finally关闭自己创建的日志handler。
def boot(entry, home, project, action, *args):
    # action/*args 原样作为参数数组传子进程，返回成功无值；子进程非零转 RuntimeError。
    # 本函数会真实执行所选动作；是否只读由 action 决定。Popen负责等待，日志handler由本函数关闭。
    from standby import rotating_logger

    folder = home / 'FC01_navigation'
    private_dir(folder)
    log = rotating_logger(folder / 'install.log', 1024 * 1024)
    try:
        command = ['/bin/bash', str(entry / 'navigation_boot.sh'), action, *map(str, args)]
        log.info('运行 %s', command)
        with subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env={**os.environ, 'HOME': str(home), 'FC01_PROJECT_ROOT': str(project)},
        ) as child:
            while True:
                block = os.read(child.stdout.fileno(), 4096)
                if not block:
                    break
                log.info('%s', block.decode(errors='replace').rstrip())
            if child.wait() != 0:
                raise RuntimeError(action + ' 失败；详见自动报告 install.log')
    finally:
        for handler in log.handlers:
            handler.close()


# 按生成候选→新构建→真实C++配置检查→被动地面观察顺序准备，返回待激活prefix。
def prepare(entry, home, project, mode='diagnostic', confirm=False):
    # entry/home/project 与 mode/confirm 分别指定包、用户、SDK和候选模式；成功返回 install Path。
    # config 写到 install-runtime.yaml，仍不是已激活的 runtime.yaml；任何阶段抛错会阻断后续步骤。
    if mode not in MODES:
        raise ValueError('运行模式必须为 diagnostic 或 flight-test')
    if mode == 'flight-test' and not confirm:
        raise ValueError('Flight Test 必须显式确认')
    folder = home / 'FC01_navigation'
    private_dir(folder)
    config = folder / 'install-runtime.yaml'
    boot(
        entry,
        home,
        project,
        'prepare-config',
        '--mode',
        mode,
        *(['--confirm-flight-test'] if mode == 'flight-test' else []),
        '--file',
        config,
    )
    key = source_key(entry, project)
    stage = staging_path(entry, key)
    print('正在离线编译 ARM64 原生控制器；单任务构建，首次需要等待。', flush=True)
    boot(
        entry, home, project, 'build', '--prefix', stage / 'install', '--build-dir', stage / 'build'
    )
    boot(
        entry,
        home,
        project,
        'check-config',
        '--mode',
        mode,
        *(['--confirm-flight-test'] if mode == 'flight-test' else []),
        '--file',
        config,
        '--prefix',
        stage / 'install',
    )
    print('编译和实际 YAML 检查通过；正在被动确认未解锁、接地、静止。', flush=True)
    boot(
        entry,
        home,
        project,
        'probe',
        '--mode',
        mode,
        *(['--confirm-flight-test'] if mode == 'flight-test' else []),
        '--file',
        folder / 'probe.json',
    )
    return stage / 'install'


# 读取服务真实FragmentPath/DropIn/MainPID等身份；不据同名文件直接假定本包拥有。
def unit_info(name, run):
    # run 是命令执行器，返回 systemctl 文本中含等号的键值字典；不写服务。
    # check=False 保留原语义：非零退出本身不抛异常，空字段不是服务已安全停止的证明。
    result = run(
        [
            '/usr/bin/systemctl',
            'show',
            name,
            '--property=FragmentPath,DropInPaths,MainPID,ActiveState,UnitFileState',
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    return dict(line.split('=', 1) for line in (result.stdout or '').splitlines() if '=' in line)


# 拒绝符号链接、外来marker、drop-in与未知活进程，所有拒绝发生在服务写入前。
def managed_unit(path, marker, info):
    # path/marker/info 共同约束候选unit来源；通过无返回值，冲突抛 RuntimeError。
    # 这是一次文件和查询快照检查，不持有systemd锁，也不保证之后服务状态不会变化。
    if path.is_symlink():
        raise RuntimeError('保留符号链接服务：' + str(path))
    if path.exists() and not path.read_text().startswith(marker + '\n'):
        raise RuntimeError('保留不属于本包的服务：' + str(path))
    if info.get('DropInPaths') or info.get('FragmentPath') not in ('', str(path), None):
        raise RuntimeError('服务存在其他来源或覆盖配置，未变更：' + str(path))
    if info.get('MainPID', '0') != '0' and not path.exists():
        raise RuntimeError('服务进程来源不明，未变更')


# 沿/proc父链验证每个控制器属于允许服务，发现非托管者保留它并拒绝第二个控制器。
def check_controllers(processes, allowed_pids):
    # processes 为进程快照，allowed_pids 为允许的服务祖先PID；通过无返回值。
    # 不能读清父链时不猜测归属；发现未归属控制器只拒绝，不向其发送停止或杀死信号。
    for process in processes:
        if not any(Path(arg).name == 'fc01_system_node' for arg in process.argv[:2]):
            continue
        pid = process.pid
        seen = set()
        while pid > 1 and pid not in seen and pid not in allowed_pids:
            seen.add(pid)
            try:
                fields = Path('/proc', str(pid), 'status').read_text().splitlines()
                pid = int(next(x.split()[1] for x in fields if x.startswith('PPid:')))
            except (OSError, ValueError, StopIteration):
                break
        if pid not in allowed_pids:
            raise RuntimeError('发现非托管控制器，保留其进程，拒绝启动第二个控制器')


# 声明本包两个只读状态窗口路径；不自动搜索/覆盖其它桌面启动器。
def desktop_paths(home):
    # 返回本包约定的两个 Path；不检查存在性、不创建目录，也不搜索其他用户入口。
    return [
        home / 'Desktop/FC01_Navigation_Status.desktop',
        home / '.config/autostart/FC01_Navigation_Status.desktop',
    ]


# 只覆盖带本包marker的普通入口，外来文件/符号链接保留。
def check_desktop(home):
    # 通过无返回值；只读检查本包约定路径，外来标记或symlink抛错，不擅自清理。
    for p in desktop_paths(home):
        if p.is_symlink() or (p.exists() and not p.read_text().startswith(DESKTOP_MARKER + '\n')):
            raise RuntimeError('保留不属于本包的桌面入口：' + str(p))


# 地面许可绑定当前boot、ok、armed未出现及2.5秒单调窗口；重启/回退时间均无效。
def ground_permit_fresh(home):
    # home 定位探针文件，返回当前boot内短期地面许可的 bool；不启动新探针。
    # JSON/文件/字段错误返回 False；bool 不当作数值时间，单调时钟回退也拒绝。
    try:
        row = json.loads((home / 'FC01_navigation/probe.json').read_text())
        stamp = row.get('updated_monotonic')
        return (
            row.get('ok') is True
            and row.get('armed_seen') is False
            and row.get('boot_id') == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            and type(stamp) in (int, float)
            and math.isfinite(stamp)
            and 0 <= time.monotonic() - stamp <= 2.5
        )
    except (OSError, ValueError, AttributeError):
        return False


# sudo输密码可无限等待，认证前旧地面证据不能用于后续写服务。
def authenticate_admin(run):
    # Password entry may take arbitrarily long. No ground authorization
    # obtained before this prompt is used to mutate a service afterward.
    # run 默认来自上层注入；只做 sudo -v 认证，非零保留执行器异常，不授予飞行权限。
    run(['sudo', '-v'], check=True)


# 许可过期或force时重新被动探测，最终复验当前时刻；未知即拒绝服务变更。
def ensure_ground(entry, home, project, force=False):
    # force=True 先重采样；通过无返回值，探针错误或复验过期直接阻断后续服务操作。
    # 不把认证前、前一任务或其他boot的成功记录当作当前持续有效的许可。
    if force or not ground_permit_fresh(home):
        boot(entry, home, project, 'probe', '--file', home / 'FC01_navigation/probe.json')
    if not ground_permit_fresh(home):
        raise RuntimeError('地面授权已过期，未继续变更服务')


# 先核验服务/进程/链接/配置身份，再认证并每步重验地面；只改托管对象，原生监督负责停止。
def deploy(
    entry,
    home,
    project,
    prefix,
    *,
    user=None,
    units=Path('/etc/systemd/system'),
    run=subprocess.run,
    processes=None,
    mode='diagnostic',
    confirm=False,
):
    # 这是旧安装器的有副作用入口：成功无返回值，units/run/processes 可注入测试现场。
    # 先只读拒绝冲突，再按原顺序认证、复验地面、停旧服务、换链接/unit并启用新服务。
    # 此函数不是完整补偿事务；中途异常可留下已完成的外部修改，正式恢复链见 navigation_restore。
    old, new = units / OLD, units / SERVICE
    info = {name: unit_info(name, run) for name in (OLD, SERVICE)}
    managed_unit(old, OLD_MARKER, info[OLD])
    managed_unit(new, MARKER, info[SERVICE])
    check_desktop(home)
    check_controllers(
        process_list() if processes is None else processes,
        {int(x.get('MainPID') or 0) for x in info.values()} - {0},
    )
    link = entry / 'install'
    pending = entry / '.install-link-new'
    if pending.exists() or pending.is_symlink():
        raise RuntimeError('待发布链接已存在，保留它')
    if link.exists() and not link.is_symlink():
        raise RuntimeError('保留非托管 install 路径')
    if link.is_symlink() and (link.resolve().parent.parent != entry / '.build'):
        raise RuntimeError('保留指向其他项目的 install 链接')
    unit = render_unit(
        user or pwd.getpwuid(os.getuid()).pw_name, home, entry, project, mode, confirm
    )
    prepared = home / 'FC01_navigation/install-runtime.yaml'
    live = home / 'FC01_navigation/runtime.yaml'
    config_changed = prepared.is_file() and (
        not live.is_file() or prepared.read_bytes() != live.read_bytes()
    )
    # 源码prefix、unit或实际运行YAML任何变化都要求安全停旧服务；不能仅以路径相同推断参数未变。
    changed = (
        not new.exists()
        or new.read_text() != unit
        or not link.is_symlink()
        or link.resolve() != prefix.resolve()
        or config_changed
    )
    with tempfile.TemporaryDirectory(prefix='fc01_navigation_unit_') as folder:
        temp = Path(folder) / SERVICE
        temp.write_text(unit)
        run(['/usr/bin/systemd-analyze', 'verify', str(temp)], check=True)
        # Every refusal above precedes sudo/service mutations.
        authenticate_admin(run)
        ensure_ground(entry, home, project, force=True)
        if old.exists():
            ensure_ground(entry, home, project)
            run(['sudo', '-n', '/usr/bin/systemctl', 'disable', '--now', OLD], check=True)
        if changed and new.exists():
            ensure_ground(entry, home, project)
            run(['sudo', '-n', '/usr/bin/systemctl', 'stop', SERVICE], check=True)
        if changed:
            ensure_ground(entry, home, project)
            pending.symlink_to(prefix)
            os.replace(pending, link)
            run(['sudo', '-n', '/usr/bin/install', '-m', '0644', str(temp), str(new)], check=True)
            run(['sudo', '-n', '/usr/bin/systemctl', 'daemon-reload'], check=True)
        for p in desktop_paths(home):
            p.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            p.write_text(render_desktop(entry))
            p.chmod(0o700)
        ensure_ground(entry, home, project)
        run(['sudo', '-n', '/usr/bin/systemctl', 'enable', '--now', SERVICE], check=True)
    print('导航服务已启用；当前终端显示状态，桌面登录后也会显示只读状态窗口。', flush=True)


# 同用户非阻塞安装锁覆盖prepare和deploy，避免两个部署竞争生成配置/切换入口。
def install(entry, home, mode='diagnostic', confirm=False):
    # 普通用户入口，成功无返回值；环境变量优先选择SDK，否则沿用历史默认项目目录。
    # with 持有本用户非阻塞文件锁直至准备/部署结束；失败不重试、不启动另一套并行部署。
    if os.getuid() == 0:
        raise RuntimeError('请用 orangepi 普通用户运行；安装服务时会请求 sudo')
    project = Path(os.environ.get('FC01_PROJECT_ROOT', str(home / PROJECT))).expanduser().resolve()
    lockdir = home / '.cache/fc01_navigation'
    private_dir(lockdir)
    with open(lockdir / 'install.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        prefix = prepare(entry, home, project, mode, confirm)
        deploy(entry, home, project, prefix, mode=mode, confirm=confirm)


# 原待机与导航服务均须确认托管；地面复验后禁用导航、恢复既有待机服务。
def rollback_services(entry, home, project, units=Path('/etc/systemd/system'), run=subprocess.run):
    # units/run 可注入替身；成功无返回值，实际会禁用导航并启用原待机服务。
    # 这里只恢复既有托管服务选择，不还原历史源码/YAML快照；每次关键变更前重验地面。
    # marker只证明本函数认可的归属，不在这里重新验证旧服务配置是否关闭全部动作门。
    for name, marker in ((OLD, OLD_MARKER), (SERVICE, MARKER)):
        p = units / name
        managed_unit(p, marker, unit_info(name, run))
        if not p.exists():
            raise RuntimeError('没有可验证的托管服务，未回滚：' + name)
    authenticate_admin(run)
    ensure_ground(entry, home, project, force=True)
    run(['sudo', '-n', '/usr/bin/systemctl', 'disable', '--now', SERVICE], check=True)
    ensure_ground(entry, home, project)
    run(['sudo', '-n', '/usr/bin/systemctl', 'enable', '--now', OLD], check=True)


# 回滚也持同一安装锁并重新探测，不能把历史成功探针当作本次地面授权。
def rollback(entry, home):
    # 解析当前SDK后持同一安装锁；先重新探测，再委托 rollback_services，成功无返回值。
    # 打印成功只在整个调用返回后执行；异常保留，不因历史已有探针而跳过本轮观测。
    project = Path(os.environ.get('FC01_PROJECT_ROOT', str(home / PROJECT))).expanduser().resolve()
    lockdir = home / '.cache/fc01_navigation'
    private_dir(lockdir)
    with open(lockdir / 'install.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        boot(entry, home, project, 'probe', '--file', home / 'FC01_navigation/probe.json')
        rollback_services(entry, home, project)
    print('已回到原关闭动作门待命服务；导航自动起飞已停用。', flush=True)
