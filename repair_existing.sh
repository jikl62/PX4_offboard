#!/usr/bin/env bash
fc01_existing_entry=${FC01_CONTROL_INSTALL_DIR:-"$HOME/FC01_OrangePi_Control_20261027"}
if FC01_EXISTING_ENTRY="$fc01_existing_entry" python3 - <<'PY'
import os
from pathlib import Path
p = Path(os.environ['FC01_EXISTING_ENTRY']) / 'fc01_control.py'
if p.is_symlink():
    raise SystemExit('拒绝修改符号链接')
s = p.read_text(encoding='utf-8')
old = 'for key in ACTION_FLAGS[:6]:'
new = 'for key in ACTION_FLAGS[1:6]:'
if s.count(old) == 1:
    fixed = s.replace(old, new, 1)
    compile(fixed, str(p), 'exec')
    try:
        with p.with_name('fc01_control.py.before_gate_fix').open('x', encoding='utf-8') as f:
            f.write(s)
    except FileExistsError:
        pass
    p.write_text(fixed, encoding='utf-8')
elif s.count(new) != 1:
    raise SystemExit('入口版本不匹配，未修改')
print('已修正：flight_enabled=false，使用独立姿态定高动作开关。')
PY
then
  bash "$fc01_existing_entry/start.sh" --flight
else
  exit 1
fi
