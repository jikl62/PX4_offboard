#!/usr/bin/env python3
"""Read-only replay of the 2026-10-11 companion CSV. Never contacts PX4."""
import csv
from pathlib import Path
csv_path=Path(__file__).resolve().parent.parent / 'evidence' / 'telemetry_20261011T034828Z.csv'
with csv_path.open(newline='',encoding='utf-8') as f:
    rows=list(csv.DictReader(f))
assert rows and len(rows)>200, 'missing original field telemetry'
first=next(i for i,r in enumerate(rows) if r['reason']=='ARMED_GROUND_NOT_STABLE')
prev=rows[first-1];r=rows[first]
print('first_logged_failure:',r['steady_s'],'phase',r['phase'])
print('previous:',prev['phase'],'vz',prev['vz_ned'],'relative_height',prev['relative_height_m'])
print('failure:',r['phase'],'armed',r['armed'],'landed',r['landed'],'vz',r['vz_ned'],
      'relative_height',r['relative_height_m'],'planned_thrust_z',r['aa_thrust_z'])
assert r['phase']=='SAFETY' and r['armed']=='1' and r['landed']=='1'
assert abs(float(r['vz_ned'])) > .1
assert abs(float(r['relative_height_m'])) < .15
assert abs(float(r['aa_thrust_z']))<1e-12
print('CSV_OBSERVATION_PASS: speed exceeds .1 at first logged fault, height within .15, landed, no planned thrust')
print('LIMITATION: sampled about once per .5s; cannot establish the actual trigger tick without Core evidence')
