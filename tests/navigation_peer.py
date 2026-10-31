#!/usr/bin/env python3
"""ROS/native boundary peer; no serial or flight effects."""
import json
from pathlib import Path
import select,signal,socket,sys,time
import yaml
config=Path(sys.argv[sys.argv.index('--params-file')+1]);p=yaml.safe_load(config.read_text())['fc01_system']['ros__parameters']
fd=int(next(x.split('=',1)[1] for x in sys.argv if x.startswith('--test-fd=')))
conn=socket.socket(fileno=fd);conn.setblocking(False)
stop=[None];signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,time.monotonic()))
buf=b'';ops=[]
while True:
    end=stop[0] is not None and time.monotonic()-stop[0]>.35
    state=dict(kind='state',phase='SAFE_DISARMED' if end else 'DESCENT' if stop[0] else 'HEIGHT_HOLD',
        reason='native recovery',status_fresh=True,local_fresh=True,land_fresh=True,armed=not end,landed=end)
    conn.sendall((json.dumps(state)+'\n').encode())
    if end:break
    if select.select([conn],[],[],.04)[0]:
        data=conn.recv(4096)
        if not data:break
        buf+=data
        while b'\n' in buf:
            line,buf=buf.split(b'\n',1);ops.append(line.decode().split()[1])
(config.parent/'trace.json').write_text(json.dumps(dict(ops=ops,rc_enabled=p['rc_navigation_enabled'],
    automatic_arm_disabled=not p['arm_enabled'],external_log_disabled='--disable-external-lib-logs' in sys.argv)))
conn.close()
