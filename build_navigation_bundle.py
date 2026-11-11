#!/usr/bin/env python3
# 正式导航源码包构建器：把当前磁盘载荷与发布清单绑定；生成的默认动作是离线构建，非在线激活。
# 内嵌HEADER字节保持原合同，本轮不重新生成dist、不执行安装器。
"""Build the current source tree into an integrity-checked offline installer."""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import subprocess
import zipfile

# 沿用既有包/控制器版本标记；不能只靠版本字符串认定源/二进制相同，仍校验逐文件SHA。
RELEASE_NAME='FC01_OrangePi_Navigation_20261110_recovery_diagnostic_v5'
DEFAULT_DIRECTORY=RELEASE_NAME
RELEASE_VERSION='20261110.7'
CONTROLLER_BUILD='FC01_RECOVERY_DIAGNOSTIC_V5_20261110'
MANIFEST='FC01_RELEASE.json'
MAX_FILES=512
MAX_BYTES=8*1024*1024
# 运行机生成的构建/激活证据和部署配置不得被当作源码载荷移植到另一设备。
RUNTIME_FILES={MANIFEST,'ARM64_BUILD_EVIDENCE.json','build-complete.json',
               'OFFLINE_DEPLOYMENT.json','ACTIVATION_STATE.json','deployment-config.json'}
CACHE_PARTS={'__pycache__','.build','build','install','.pytest_cache'}

# Keep this navigation entry independent of the control/autostart installers.
# 解包载荷先验相邻整包SHA、归档SHA、CRC与512文件/8MiB/每项1MiB限制，再核对全部manifest。
# 拒绝绝对/穿越/重复/链接路径；既有目标必须逐字节一致，新目录通过RENAME_NOREPLACE原子发布。
# 原stdin经fd3保留；extract-only不运行部署，默认offline，显式activate仍需部署器合同确认。
# 此大字符串保留原字节，避免中文化修改生成的shell/Python行为和现有SHA语义。
HEADER=r'''#!/usr/bin/env bash
# Source-only release. Default action builds offline and never activates a service.
set -euo pipefail
fc01_package_self="${BASH_SOURCE[0]}"
/usr/bin/python3 - "$fc01_package_self" "$@" 3<&0 <<'FC01_UNPACK_PY'
import base64, ctypes, errno, hashlib, io, json, os, re, shutil
from pathlib import Path, PurePosixPath
import stat, sys, tempfile, time, traceback, zipfile

MANIFEST='FC01_RELEASE.json'
extract_log=None

def report(message):
    if extract_log is not None:
        with extract_log.open('a') as output:output.write(message+'\n')
    print(message,flush=True)

def fail(message):
    raise RuntimeError(message)

def safe_relative(name):
    path=PurePosixPath(name)
    return bool(name) and not path.is_absolute() and '..' not in path.parts and '\\' not in name and name==path.as_posix()

def check_ancestors(path):
    for parent in (path,*path.parents):
        if parent.is_symlink():
            fail('安装路径包含符号链接，未写入: '+str(parent))
        if parent.exists() and not parent.is_dir():
            fail('安装路径被普通文件占用: '+str(parent))

def publish_directory(stage,target):
    # Linux RENAME_NOREPLACE prevents a concurrent installer from replacing
    # even an empty existing directory after our initial inspection.
    libc=ctypes.CDLL(None,use_errno=True)
    rename=getattr(libc,'renameat2',None)
    if rename is None:
        fail('系统缺少原子独立目录发布接口 renameat2，未覆盖安装目录')
    rename.argtypes=[ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint]
    rename.restype=ctypes.c_int
    if rename(-100,os.fsencode(stage),-100,os.fsencode(target),1):
        code=ctypes.get_errno()
        if code==errno.EEXIST:
            fail('安装目录已由其他进程创建，未覆盖: '+str(target))
        raise OSError(code,os.strerror(code),str(target))

def install():
    package=Path(sys.argv[1]).absolute()
    blob=package.read_bytes()
    sidecar=Path(str(package)+'.sha256')
    if sidecar.exists() or sidecar.is_symlink():
        if sidecar.is_symlink() or not sidecar.is_file():
            fail('相邻 SHA256 文件无效: '+str(sidecar))
        lines=[line.strip() for line in sidecar.read_text().splitlines() if line.strip()]
        if len(lines)!=1:
            fail('相邻 SHA256 文件格式无效')
        fields=lines[0].split(maxsplit=1)
        if not re.fullmatch(r'[0-9a-fA-F]{64}',fields[0]):
            fail('相邻 SHA256 文件格式无效')
        if len(fields)>1 and fields[1].lstrip('*')!=package.name:
            fail('相邻 SHA256 文件不属于本安装包')
        if hashlib.sha256(blob).hexdigest()!=fields[0].lower():
            fail('安装包文件 SHA256 不匹配，未写入')
    marker=b'\n__FC01_PAYLOAD_BASE64__\n'
    if blob.count(marker)!=1:
        fail('安装包分隔符损坏')
    payload=base64.b64decode(b''.join(blob.split(marker,1)[1].split()),validate=True)
    if hashlib.sha256(payload).hexdigest()!='__ARCHIVE_SHA256__':
        fail('安装包内容 SHA256 不匹配，未写入')
    files={};total=0
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        if len(archive.infolist())>512:
            fail('安装包文件数量超限')
        for item in archive.infolist():
            name=item.filename;mode=item.external_attr>>16
            if (not safe_relative(name) or item.is_dir() or stat.S_ISLNK(mode) or
                    stat.S_IFMT(mode) not in (0,stat.S_IFREG) or name in files):
                fail('不安全的安装项: '+name)
            total+=item.file_size
            if item.file_size>1048576 or total>8388608:
                fail('安装包体积超限')
            files[name]=archive.read(item)
    if MANIFEST not in files or 'navigation.sh' not in files:
        fail('安装包缺少版本清单或 navigation.sh')
    release=json.loads(files[MANIFEST])
    if (release.get('release_name')!='__RELEASE_NAME__' or release.get('version')!='__RELEASE_VERSION__' or
            release.get('controller_build')!='__CONTROLLER_BUILD__'):
        fail('安装包版本标识不匹配')
    hashes=release.get('payload_sha256')
    expected={name:hashlib.sha256(data).hexdigest() for name,data in files.items() if name!=MANIFEST}
    if not isinstance(hashes,dict) or hashes!=expected:
        fail('版本清单与安装内容摘要不一致')
    sources=release.get('source_sha256')
    required=[name for name in expected if name.startswith('native/src/')]
    if not isinstance(sources,dict) or any(sources.get(name)!=expected[name] for name in required):
        fail('原生源码摘要清单不完整')
    args=sys.argv[2:]
    extract_only=args in (['--install-only'],['--extract-only'],['extract-only'])
    if any(arg in ('--install-only','--extract-only','extract-only') for arg in args) and not extract_only:
        fail('仅解包选项不能与部署参数混用')
    if not extract_only and 'navigation_deploy.sh' not in files:
        fail('安装包缺少 navigation_deploy.sh；旧源包请使用 --install-only')
    target=Path(os.environ.get('FC01_NAVIGATION_INSTALL_DIR',str(Path.home()/'__DIRECTORY_NAME__'))).expanduser().absolute()
    check_ancestors(target)
    if target.exists():
        if target.stat().st_uid!=os.getuid() or target.stat().st_mode&0o022:
            fail('安装目录所有者/权限不安全: '+str(target))
        for name,data in files.items():
            dest=target/name;check_ancestors(dest.parent)
            if dest.is_symlink() or not dest.is_file() or dest.read_bytes()!=data:
                fail('现有目录与本包不一致，未覆盖: '+str(dest))
        report('FC01_PAYLOAD_VERIFIED_REUSED entry='+str(target))
    else:
        target.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
        stage=Path(tempfile.mkdtemp(prefix='.'+target.name+'.unpack-',dir=target.parent))
        try:
            for name,data in files.items():
                dest=stage/name;dest.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
                with dest.open('xb') as output:
                    output.write(data);output.flush();os.fsync(output.fileno())
                dest.chmod(0o700 if name.endswith('.sh') else 0o600)
            publish_directory(stage,target)
        finally:
            if stage.exists():shutil.rmtree(stage)
        report('FC01_PAYLOAD_INSTALLED entry='+str(target))
    if extract_only:
        report('FC01_EXTRACT_ONLY_COMPLETE entry='+str(target))
        return
    report('FC01_PACKAGE_DEPLOY action='+str((args or ['offline'])[0])+' extract_log='+str(extract_log))
    os.dup2(3,0);os.close(3)
    os.execv('/bin/bash',['bash',str(target/'navigation_deploy.sh'),*(args or ['offline'])])

try:
    log_directory=Path.home()/'FC01_navigation/deploy-logs/__RELEASE_NAME__'
    check_ancestors(log_directory)
    log_directory.mkdir(mode=0o700,parents=True,exist_ok=True)
    extract_log=log_directory/(time.strftime('%Y%m%dT%H%M%S')+'-'+str(os.getpid())+'-extract.log')
    with extract_log.open('x') as output:
        output.write('FC01_PACKAGE_BEGIN package='+sys.argv[1]+'\n')
    extract_log.chmod(0o600)
    report('FC01_PACKAGE_LOG='+str(extract_log))
    install()
except Exception as error:
    if extract_log is not None:
        with extract_log.open('a') as output:output.write(traceback.format_exc())
    print('FC01_PACKAGE_FAILED stage=integrity_or_extract exit=1 error='+str(error)+
          ' log='+str(extract_log or 'unavailable'),file=sys.stderr,flush=True)
    sys.exit(1)
FC01_UNPACK_PY
exit $?
__FC01_PAYLOAD_BASE64__
'''

# 按明确后缀与native/tests/docs递归收当前文件，排除缓存/运行态和独立receive A/B；大小/链接不符即拒绝。
def names(root):
    """Curated source paths from disk, including uncommitted and new files."""
    root=Path(root)
    roots=[p for p in root.iterdir() if p.is_file() and p.suffix in ('.py','.sh','.md','.json')
           and p.name not in RUNTIME_FILES]
    roots.extend(p for directory in ('native','tests','docs') for p in (root/directory).rglob('*')
                 if p.is_file() and not (set(p.relative_to(root).parts)&CACHE_PARTS)
                 and not p.is_relative_to(root/'native/tests/ros_receive_ab'))
    result=sorted(str(p.relative_to(root)) for p in roots)
    if len(result)+1>MAX_FILES or sum((root/p).stat().st_size for p in result)>MAX_BYTES:
        raise RuntimeError('Curated source payload exceeds extractor bounds')
    for name in result:
        if (root/name).is_symlink():
            raise ValueError('Source payload cannot contain a symlink: '+name)
    return result

# 摘要实际磁盘字节，记录验证时Git基线与dirty；清单自身排除以避免自引用。
def make_manifest(root):
    """Describe actual payload bytes; callers may add verification metadata."""
    root=Path(root)
    hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names(root)}
    # Git不可用时保留None，不伪造提交身份；不会执行push/checkout或修改仓库。
    def git(*args):
        result=subprocess.run(['git','-C',str(root),*args],capture_output=True,text=True)
        return result.stdout.strip() if result.returncode==0 else None
    return {'release_name':RELEASE_NAME,'version':RELEASE_VERSION,'controller_build':CONTROLLER_BUILD,
            'source_commit':git('rev-parse','HEAD'),'working_tree_dirty':bool(git('status','--porcelain')),
            'source_sha256':dict(hashes),'payload_sha256':hashes}

# 已存在清单必须与当前完整载荷/原生源SHA一致；不在打包时静默覆盖过期验收记录。
def _release_bytes(root,files):
    path=root/MANIFEST
    if path.is_symlink():
        raise ValueError('FC01_RELEASE.json cannot be a symlink')
    if path.exists():
        data=path.read_bytes();release=json.loads(data)
        if any(release.get(key)!=value for key,value in
               (('release_name',RELEASE_NAME),('version',RELEASE_VERSION),('controller_build',CONTROLLER_BUILD))):
            raise ValueError('FC01_RELEASE.json release/version/controller identity differs from this release')
        hashes={name:hashlib.sha256(content).hexdigest() for name,content in files.items()}
        if release.get('payload_sha256')!=hashes:
            raise ValueError('FC01_RELEASE.json payload_sha256 differs from the current source tree')
        sources=release.get('source_sha256')
        if not isinstance(sources,dict) or any(sources.get(name)!=hashes[name] for name in hashes if name.startswith('native/src/')):
            raise ValueError('FC01_RELEASE.json lacks current native/src source hashes')
        if any(name not in hashes or hashes[name]!=value for name,value in sources.items()):
            raise ValueError('FC01_RELEASE.json source_sha256 differs from the current source tree')
        return data
    return (json.dumps(make_manifest(root),indent=2,sort_keys=True)+'\n').encode()

# 目录名只能是单层安全标识；固定ZIP时间与权限、嵌入摘要，再输出整包sidecar，不激活任何服务。
def build(output,source=None,*,directory_name=DEFAULT_DIRECTORY):
    if not directory_name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in directory_name):
        raise ValueError('directory_name must be a plain directory name')
    root=Path(source) if source else Path(__file__).resolve().parent
    files={name:(root/name).read_bytes() for name in names(root)}
    files[MANIFEST]=_release_bytes(root,files)
    if len(files)>MAX_FILES or sum(map(len,files.values()))>MAX_BYTES or any(len(data)>1048576 for data in files.values()):
        raise RuntimeError('Source payload or release manifest exceeds extractor bounds')
    stream=io.BytesIO()
    with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,data in sorted(files.items()):
            info=zipfile.ZipInfo(name,date_time=(2026,11,10,0,0,0))
            info.create_system=3;info.external_attr=0o100600<<16;info.compress_type=zipfile.ZIP_DEFLATED
            archive.writestr(info,data)
    payload=stream.getvalue()
    header=(HEADER.replace('__ARCHIVE_SHA256__',hashlib.sha256(payload).hexdigest())
            .replace('__DIRECTORY_NAME__',directory_name).replace('__RELEASE_NAME__',RELEASE_NAME)
            .replace('__RELEASE_VERSION__',RELEASE_VERSION).replace('__CONTROLLER_BUILD__',CONTROLLER_BUILD))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    data=header.encode()+base64.encodebytes(payload)
    output.write_bytes(data);output.chmod(0o700)
    Path(str(output)+'.sha256').write_text(hashlib.sha256(data).hexdigest()+'  '+output.name+'\n')
    return output

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--directory-name',default=DEFAULT_DIRECTORY)
    args=parser.parse_args()
    print(build(args.output,directory_name=args.directory_name))
