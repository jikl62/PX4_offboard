#!/usr/bin/env python3
"""Reproducible offline source installer; no host-built binary is shipped."""
import base64
import hashlib
import io
from pathlib import Path
import sys
import zipfile
from build_bundle import HEADER

def names(root):
    roots=[p for p in root.iterdir() if p.is_file() and p.suffix in ('.py','.sh','.md','.json')]
    roots.extend(p for directory in ('native','tests','docs') for p in (root/directory).rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts)
    result=sorted(str(p.relative_to(root)) for p in roots)
    if len(result)>300 or sum((root/p).stat().st_size for p in result)>4194304:
        raise RuntimeError('Curated source payload exceeds extractor bounds')
    return result

def build(output,source=None):
    root=Path(source) if source else Path(__file__).resolve().parent
    stream=io.BytesIO()
    with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as z:
        for name in names(root):
            info=zipfile.ZipInfo(name,date_time=(2026,10,28,0,0,0))
            info.create_system=3;info.external_attr=0o100600<<16;info.compress_type=zipfile.ZIP_DEFLATED
            z.writestr(info,(root/name).read_bytes())
    payload=stream.getvalue()
    header=(HEADER.replace('__ARCHIVE_SHA256__',hashlib.sha256(payload).hexdigest())
        .replace('FC01_CONTROL_INSTALL_DIR','FC01_NAVIGATION_INSTALL_DIR')
        .replace('FC01_OrangePi_Control_20261027_R2','FC01_OrangePi_Navigation_20261028')
        .replace('start.sh','navigation.sh'))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_bytes(header.encode()+base64.encodebytes(payload));output.chmod(0o700)
    return output

if __name__=='__main__':print(build(sys.argv[1]))
