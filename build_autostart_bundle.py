#!/usr/bin/env python3
# 独立待机安装包的构建入口：复用控制安装器的校验解包头，载荷仅来自显式文件清单。
# 此程序生成文件，不连接飞控、不安装服务；实际待机生命周期由 autostart/standby 处理。
"""Build a separate offline standby installer with the existing verified extractor."""
import base64
import hashlib
import io
from pathlib import Path
import sys
import zipfile

from build_bundle import HEADER

# 显式载荷同时包含启动脚本、运行代码、离线测试和对应设计记录，避免把工作目录任意文件装入包。
NAMES = (
    'autostart.sh',
    'autostart.py',
    'standby.sh',
    'standby.py',
    'launcher.py',
    'fc01_control.py',
    'README_AUTOSTART_zh.md',
    'build_bundle.py',
    'build_autostart_bundle.py',
    'VERIFICATION_AUTOSTART.md',
    'tests/standby_native.py',
    'tests/standby_peer.py',
    'tests/test_standby.py',
    'tests/test_autostart.py',
    'tests/fixtures/base.yaml',
    'docs/superpowers/specs/2026-10-27-orangepi-autostart-design.md',
    'docs/superpowers/plans/2026-10-27-orangepi-autostart.md',
)


# 把 source 目录打成确定时间戳的 ZIP，再写入带归档 SHA256 的自解包脚本；返回输出路径。
def build(output, source=None):
    root = Path(source) if source else Path(__file__).resolve().parent
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in NAMES:
            # 固定 ZIP 元数据使相同输入可重复生成相同载荷；默认文件权限不授予群组或其他用户写权。
            info = zipfile.ZipInfo(name, date_time=(2026, 10, 27, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100600 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, (root / name).read_bytes())
    payload = stream.getvalue()
    # 替换的是安装目录环境变量和入口标识；没有在构建阶段执行新生成的安装器。
    header = (
        HEADER.replace('__ARCHIVE_SHA256__', hashlib.sha256(payload).hexdigest())
        .replace('FC01_CONTROL_INSTALL_DIR', 'FC01_AUTOSTART_INSTALL_DIR')
        .replace('FC01_OrangePi_Control_20261027_R2', 'FC01_OrangePi_Autostart_20261027')
        .replace('start.sh', 'autostart.sh')
    )
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header.encode() + base64.encodebytes(payload))
    output.chmod(0o700)
    return output


if __name__ == '__main__':
    print(build(sys.argv[1]))
