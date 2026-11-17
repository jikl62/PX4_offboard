#!/usr/bin/env python3
# 历史交互控制源包构建器：固定ZIP元数据与SHA校验头，保留操作者原stdin用于后续显式控制台。
# HEADER是已测解包载荷，保持字符串字节不变；下方中文解释其路径/权限/完整性和发布合同。
"""Build an offline, integrity-checked installer with the operator's stdin intact."""
import argparse
import base64
import hashlib
import io
from pathlib import Path
import zipfile


# 内嵌脚本先校验base64/归档SHA/CRC、路径穿越/链接/文件数和4MiB总量，再检查整个既有树。
# 目标祖先不能为symlink；不同现有文件不覆盖，新文件经fsync+link发布；--install-only禁止混运行参数。
# 默认解包成功后恢复fd3保存的原stdin，再exec原start.sh；构建函数本身绝不执行此载荷。
HEADER = r'''#!/usr/bin/env bash
# FC01 Orange Pi control entry. Offline; original FC01 installation is preserved.
set -e
fc01_self="${BASH_SOURCE[0]}"
/usr/bin/python3 - "$fc01_self" "$@" 3<&0 <<'FC01_UNPACK_PY'
import base64, hashlib, io, os
from pathlib import Path, PurePosixPath
import stat, sys, tempfile, zipfile

def fail(text):
    raise RuntimeError(text)

def check_ancestors(path):
    for p in (path, *path.parents):
        if p.is_symlink():
            fail('安装路径包含符号链接，未写入: '+str(p))
        if p.exists() and not p.is_dir():
            fail('安装路径被普通文件占用: '+str(p))

def install():
    blob = Path(sys.argv[1]).read_bytes()
    marker = b'\n__FC01_PAYLOAD_BASE64__\n'
    if blob.count(marker) != 1:
        fail('安装包分隔符损坏')
    encoded = blob.split(marker, 1)[1]
    payload = base64.b64decode(b''.join(encoded.split()), validate=True)
    if hashlib.sha256(payload).hexdigest() != '__ARCHIVE_SHA256__':
        fail('安装包 SHA256 不匹配，未写入')
    files = {}
    with zipfile.ZipFile(io.BytesIO(payload)) as z:
        # Navigation bundles include explicit diagnostic/flight-test candidate
        # configs in addition to the historical source set.
        if len(z.infolist()) > 320:
            fail('安装包文件数量超限')
        total = 0
        for item in z.infolist():
            name = item.filename
            path = PurePosixPath(name)
            mode = item.external_attr >> 16
            if (not name or path.is_absolute() or '..' in path.parts or '\\' in name
                    or name != path.as_posix() or item.is_dir()
                    or stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG))
                    or name in files):
                fail('不安全的安装项: '+name)
            total += item.file_size
            if item.file_size > 1048576 or total > 4194304:
                fail('安装包体积超限')
            files[name] = z.read(item)  # CRC checked before writing anything.
    if 'start.sh' not in files:
        fail('安装包缺少 start.sh')
    target = Path(os.environ.get('FC01_CONTROL_INSTALL_DIR', str(Path.home()/'FC01_OrangePi_Control_20261027_R2'))).expanduser().absolute()
    check_ancestors(target)
    if target.exists() and (target.stat().st_uid != os.getuid() or target.stat().st_mode & 0o022):
        fail('安装目录所有者/权限不安全: '+str(target))
    # Inspect the complete existing tree before writing any payload file.
    for name, data in files.items():
        dest = target/name
        check_ancestors(dest.parent)
        if dest.is_symlink():
            fail('保留现有符号链接，未安装: '+str(dest))
        if dest.exists() and (not dest.is_file() or dest.read_bytes() != data):
            fail('保留现有不同文件，未覆盖: '+str(dest))
    target.mkdir(mode=0o700, parents=True, exist_ok=True)
    for name, data in files.items():
        dest = target/name
        if dest.exists():
            continue
        dest.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        # Publish complete files without replacing an existing pathname.
        fd, temp_name = tempfile.mkstemp(prefix='.fc01_', dir=dest.parent)
        try:
            with os.fdopen(fd, 'wb') as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
            os.chmod(temp_name, 0o700 if name.endswith('.sh') else 0o600)
            os.link(temp_name, dest)
        finally:
            os.unlink(temp_name)
    print('运行入口已就绪: '+str(target/'start.sh'), flush=True)
    args = sys.argv[2:]
    if '--install-only' in args:
        if args != ['--install-only']:
            fail('--install-only 不能与运行参数混用')
        return
    # The Python program itself came from a heredoc. Restore the original TTY
    # (or caller's pipe) before executing the live operator console.
    os.dup2(3, 0)
    os.close(3)
    os.execv('/bin/bash', ['bash', str(target/'start.sh'), *args])

try:
    install()
except Exception as exc:
    print('安装未完成: '+str(exc), file=sys.stderr, flush=True)
    sys.exit(1)
FC01_UNPACK_PY
exit $?
__FC01_PAYLOAD_BASE64__
'''


# 用归档字节SHA替换唯一占位再拼base64；源文本只做序列化，不运行shell。
def render_archive(payload):
    header = HEADER.replace('__ARCHIVE_SHA256__', hashlib.sha256(payload).hexdigest())
    return header.encode('utf8') + base64.encodebytes(payload)


# 从显式旧入口与测试/文档文件清单打包，固定时间/权限以便重复生成；返回生成路径。
def build(output, source=None):
    root = Path(source) if source is not None else Path(__file__).resolve().parent
    names = [
        'fc01_control.py',
        'launcher.py',
        'start.sh',
        'repair_existing.sh',
        'build_bundle.py',
        'README_zh.md',
        'VERIFICATION.md',
    ]
    names += sorted(
        str(p.relative_to(root))
        for directory in ('tests', 'docs')
        for p in (root / directory).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py', '.yaml', '.md')
    )
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in names:
            info = zipfile.ZipInfo(name, date_time=(2026, 10, 26, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100600 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, (root / name).read_bytes())
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(render_archive(stream.getvalue()))
    output.chmod(0o700)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(build(args.output))
