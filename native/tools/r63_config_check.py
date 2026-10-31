#!/usr/bin/env python3
"""Fail closed if R6.3 candidate config silently enables actual flight."""
import pathlib
import re
import sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
def scalar(name):
    m = re.search(r'^\s*' + re.escape(name) + r':\s*([^#\n]+)', text, re.M)
    if not m:
        raise SystemExit('MISSING_CONFIG_KEY: ' + name)
    return m.group(1).strip()
assert abs(float(scalar('aa_hover_thrust')) - 0.475) < 1e-12
for key in ('flight_enabled', 'flight_actions_enabled', 'arm_enabled',
            'takeoff_enabled', 'flight_parameters_confirmed',
            'bench_arm_only_enabled'):
    assert scalar(key) == 'false', f'UNSAFE_DEFAULT: {key}={scalar(key)}'
print('R63_CONFIG_0P475_AND_DEFAULT_GATES_CLOSED_PASS')
