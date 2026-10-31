#!/usr/bin/env python3
"""Build a separate offline standby installer with the existing verified extractor."""
import base64
import hashlib
import io
from pathlib import Path
import sys
import zipfile

from build_bundle import HEADER

NAMES = ('autostart.sh', 'autostart.py', 'standby.sh', 'standby.py',
         'launcher.py', 'fc01_control.py', 'README_AUTOSTART_zh.md',
         'build_bundle.py', 'build_autostart_bundle.py', 'VERIFICATION_AUTOSTART.md',
         'tests/standby_native.py', 'tests/standby_peer.py', 'tests/test_standby.py',
         'tests/test_autostart.py', 'tests/fixtures/base.yaml',
         'docs/superpowers/specs/2026-10-27-orangepi-autostart-design.md',
         'docs/superpowers/plans/2026-10-27-orangepi-autostart.md')

def build(output, source=None):
    root = Path(source) if source else Path(__file__).resolve().parent
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as z:
        for name in NAMES:
            info = zipfile.ZipInfo(name, date_time=(2026, 10, 27, 0, 0, 0))
            info.create_system = 3
            info.external_attr = 0o100600 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, (root/name).read_bytes())
    payload = stream.getvalue()
    header = (HEADER.replace('__ARCHIVE_SHA256__', hashlib.sha256(payload).hexdigest())
              .replace('FC01_CONTROL_INSTALL_DIR', 'FC01_AUTOSTART_INSTALL_DIR')
              .replace('FC01_OrangePi_Control_20261027_R2', 'FC01_OrangePi_Autostart_20261027')
              .replace('start.sh', 'autostart.sh'))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header.encode()+base64.encodebytes(payload))
    output.chmod(0o700)
    return output

if __name__ == '__main__':
    print(build(sys.argv[1]))
