#!/usr/bin/env python3
"""Read-only SHA/ZIP verification for the two extracted handoff packages."""
from pathlib import Path
import zipfile,hashlib,re
ROOT=Path(__file__).resolve().parent.parent
DATA=ROOT/'input'/'data';STARTER=ROOT/'input'/'starter'
MAN=DATA/'META'/'SHA256_MANIFEST.txt'
checked=0;bad=[]
for line in MAN.read_text().splitlines():
 m=re.match(r'([0-9a-f]{64})\s+(.+?)\s+#\s+\d+\s+bytes$',line)
 if not m:bad.append(('bad_manifest_line',line));continue
 digest,rel=m.groups();path=DATA/rel
 if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:bad.append(('digest_mismatch',rel))
 checked+=1
print('S10_DATA_MANIFEST',checked,'verified',len(bad),'mismatches',bad[:10])
with zipfile.ZipFile(STARTER/'R4_BASELINE_REFERENCE'/'R4_SOURCE_original.zip') as a,zipfile.ZipFile(DATA/'BASELINE_R4'/'R4_SOURCE_0.4.4-offboard-flight-dev-r4(1).zip') as b:
 va={x.filename:hashlib.sha256(a.read(x.filename)).hexdigest() for x in a.infolist() if not x.is_dir()}
 vb={x.filename:hashlib.sha256(b.read(x.filename)).hexdigest() for x in b.infolist() if not x.is_dir()}
 print('R4_ARCHIVES_MATCH',va==vb,'files',len(va),'starter_zip_crc',a.testzip(),'data_zip_crc',b.testzip())
R4=ROOT/'input'/'R4_original'/'0.4.4-offboard-flight-dev-r4'/'src'/'jikl_fc01_system'
S10=DATA/'S10_MODIFIED_SOURCE'/'src'/'jikl_fc01_system'
ra={p.relative_to(R4).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in R4.rglob('*') if p.is_file()}
rb={p.relative_to(S10).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in S10.rglob('*') if p.is_file()}
changes=sorted(x for x in set(ra)|set(rb) if ra.get(x)!=rb.get(x))
print('SOURCE_CHANGED_FILES',len(changes));print('\n'.join(changes))
assert not bad and va==vb and len(changes)==2
