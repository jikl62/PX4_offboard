# Independent ULog cross-check, 2026-11-16

Independently parsed all 12 `.ulg` files directly in `/home/jikl62` and `/home/jikl62/桌面` with pyulog. No archives, hardware, SSH, ROS, source edits, or companion-log time alignment were used. Each file is treated as an independent recording. Filename dates are labels and are not verified clock evidence.

All 12 file hashes, durations, firmware hashes, observed nav-state sets, final Offboard heartbeat timestamps, z/vz/z_deriv extrema, vertical reset transitions, and dropout counts match `/tmp/fc01-stage1-ulog-audit/audit.json`. No blocking discrepancy was found in these checked outputs.

No recording contains Position mode (`nav_state=2`). Six contain historical Offboard mode (`nav_state=14`) and logged `offboard_control_mode` samples. These historical recordings cannot establish the current Position height baseline.

| File | Duration s | Observed nav states | Offboard samples | Last heartbeat boot s |
| --- | ---: | --- | ---: | ---: |
| /home/jikl62/log_0_2026-10-18-21-02-18_1.ulg | 14.403929 | 1, 14, 15 | 118 | 481.034862 |
| /home/jikl62/log_0_2026-10-20-10-02-06.ulg | 11.035125 | 1, 14 | 41 | 246.871196 |
| /home/jikl62/log_0_2026-10-20-11-12-26_1.ulg | 27.549694 | 1, 15 | 0 | absent |
| /home/jikl62/log_2_2026-10-16-11-57-46_1.ulg | 23.417627 | 1, 15 | 0 | absent |
| /home/jikl62/log_4_2026-10-16-11-58-42_1.ulg | 43.550467 | 15 | 0 | absent |
| /home/jikl62/log_5_2026-10-16-12-00-18_1.ulg | 30.125789 | 1, 15 | 0 | absent |
| /home/jikl62/桌面/log_0_2026-10-20-11-19-52.ulg | 16.403305 | 1, 14 | 108 | 209.114193 |
| /home/jikl62/桌面/log_10_2026-11-15-16-47-56.ulg | 7.677352 | 15 | 0 | absent |
| /home/jikl62/桌面/log_1_2026-10-20-11-20-32.ulg | 0.991693 | 15 | 0 | absent |
| /home/jikl62/桌面/log_2_2026-10-20-11-21-14.ulg | 17.481903 | 1, 14, 15 | 118 | 290.347726 |
| /home/jikl62/桌面/log_3_2026-10-20-11-22-20.ulg | 14.764902 | 1, 14, 15 | 121 | 360.039180 |
| /home/jikl62/桌面/log_4_2026-10-20-11-23-54.ulg | 19.293090 | 1, 14 | 123 | 449.878727 |

The November 15 filename recording is Stabilized (`nav_state=15`) throughout; `flag_control_offboard_enabled=0` and no `offboard_control_mode` topic is recorded. Its timed console messages include DDS initial-agent-ping exhaustion at 65.835730 s, session-setup failure at 66.836129 s, and serial reinitialization at 66.836591 s. This supports a failed DDS session during this recording and does not prove physical link silence or the current hardware state.

In that recording, local z goes from +0.096944585 m to -1.485310912 m; vz ranges from -0.519622624 to +0.000270103 m/s. z/vz validity stays true; z/vz reset counters remain 1 and attitude reset counter remains 2 over recorded samples. These observations characterize the estimator output; they do not distinguish actual ascent from estimation drift without an independent height observation. NED z is positive downward and its origin is local, so z cannot be read as height above ground. The full-log range also includes samples after RC disarm at 69.672397 s.

One interpretation detail was reported to the audit author: `local_summary(mask)` included whole-log reset transitions in per-nav-interval summaries. Those reset fields should either be filtered to the interval or labeled as global. The all-log reset evidence itself is correct.

Reproduction: `python3 /tmp/fc01-stage1-ulog-independent/independent_inspect.py`. The independent parser writes `independent_crosscheck.json` in the same directory.
