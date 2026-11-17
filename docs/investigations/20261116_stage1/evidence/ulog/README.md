# FC01 stage 1: read-only ULog evidence, 2026-11-16

**No Position baseline is demonstrated.** All 12 directly available candidate ULogs parse successfully, and none contains `vehicle_status.nav_state=2` (Position). Six older recordings contain both `nav_state=14` (Offboard) and logged `offboard_control_mode` samples. The November 15 filename recording contains only `nav_state=15` (Stabilized), with DDS session failure and no recorded Offboard heartbeat.

The audit treats every file as an independent recording. It does not correlate November 14 companion logs with the November 15 filename PX4 recording. No repo files, source ULogs, hardware, services, serial ports, SSH sessions, or flight parameters were changed.

## Scope and method

The source set is exactly `*.ulg` directly inside `/home/jikl62` and `/home/jikl62/桌面`; archives and deeper directories were excluded. Actual recorded `nav_state` determines classification. Filename dates are labels, not independently verified time or flight identity. All 12 firmware hashes are `07983bfb894b9372d7520d43ac10dc792d0b4b9f`, and hardware reports `DAMIAO_DM_FC01`; a common firmware hash does not establish that the same parameters, companion binary, sensors, or physical setup were used.

Mode mapping: `1=ALTCTL`, `2=POSCTL`, `14=OFFBOARD`, `15=STAB`. Arming states: `1=disarmed`, `2=armed`. ULog timestamps are PX4 boot-relative microseconds. Relative t is measured from the ULog header start; some buffered status/events precede that header and intentionally retain negative t. A mode interval ends at the next recorded transition or the last timestamp in the fully parsed ULog. Logged sample frequency is not UART throughput or actual companion publication frequency.

Local z/vz/z_deriv use NED coordinates: downward is positive; z is relative to a local estimator origin and is not physical height above ground. `z_valid` and `v_z_valid` are estimator validity flags. Counter changes identify observed estimator reference resets, and constant counters only establish no observed change within the logged window.

## Per-recording classification

`H` means `/home/jikl62/`; `D` means `/home/jikl62/桌面/`. Duration is ULog header start to the last timestamp after parsing all topics. Last heartbeat is the final logged `offboard_control_mode.timestamp`, in PX4 boot microseconds. Full nav and arming transitions with timestamped ranges are in each JSON.

| Source | Duration s | Recorded nav transitions | Last heartbeat us | z range m | vz range m/s | z_deriv range m/s |
| --- | ---: | --- | ---: | --- | --- | --- |
| H/[log_0_2026-10-18-21-02-18_1.ulg](home_log_0_2026-10-18-21-02-18_1.json) | 14.403929 | OFFBOARD → ALTCTL → STAB | 481034862 | 0.235315 to 2.013064 | -1.090570 to 0.147994 | -0.627665 to 0.181849 |
| H/[log_0_2026-10-20-10-02-06.ulg](home_log_0_2026-10-20-10-02-06.json) | 11.035125 | OFFBOARD → ALTCTL → OFFBOARD | 246871196 | -1.644088 to -0.677233 | -0.714556 to 0.798038 | -0.766147 to 0.722032 |
| H/[log_0_2026-10-20-11-12-26_1.ulg](home_log_0_2026-10-20-11-12-26_1.json) | 27.549694 | STAB → ALTCTL → STAB | absent | -3.298502 to -1.201258 | -0.615495 to 0.007205 | -0.701711 to 0.398776 |
| H/[log_2_2026-10-16-11-57-46_1.ulg](home_log_2_2026-10-16-11-57-46_1.json) | 23.417627 | STAB → ALTCTL → STAB | absent | -1.232724 to 11.518554 | -1.120692 to 8.347066 | -1.798093 to 3.029707 |
| H/[log_4_2026-10-16-11-58-42_1.ulg](home_log_4_2026-10-16-11-58-42_1.json) | 43.550467 | STAB | absent | -0.315301 to 6.142019 | 0.175135 to 4.080905 | -0.424945 to 1.008834 |
| H/[log_5_2026-10-16-12-00-18_1.ulg](home_log_5_2026-10-16-12-00-18_1.json) | 30.125789 | STAB → ALTCTL → STAB | absent | -0.584473 to 4.285823 | 0.109028 to 5.228731 | -0.530134 to 2.904124 |
| D/[log_0_2026-10-20-11-19-52.ulg](desktop_log_0_2026-10-20-11-19-52.json) | 16.403305 | OFFBOARD → ALTCTL → OFFBOARD | 209114193 | -1.322503 to 0.733745 | -0.426106 to 0.019234 | -0.438498 to 0.282801 |
| D/[log_10_2026-11-15-16-47-56.ulg](desktop_log_10_2026-11-15-16-47-56.json) | 7.677352 | STAB | absent | -1.485311 to 0.096945 | -0.519623 to 0.000270 | -0.504989 to -0.002547 |
| D/[log_1_2026-10-20-11-20-32.ulg](desktop_log_1_2026-10-20-11-20-32.json) | 0.991693 | STAB | absent | 0.855615 to 0.859598 | 0.017522 to 0.020822 | 0.008482 to 0.012989 |
| D/[log_2_2026-10-20-11-21-14.ulg](desktop_log_2_2026-10-20-11-21-14.json) | 17.481903 | OFFBOARD → ALTCTL → STAB | 290347726 | -1.587404 to 0.960643 | -0.505984 to 0.034159 | -0.480021 to 0.307877 |
| D/[log_3_2026-10-20-11-22-20.ulg](desktop_log_3_2026-10-20-11-22-20.json) | 14.764902 | OFFBOARD → ALTCTL → STAB | 360039180 | -1.559153 to 0.759085 | -0.522757 to 0.032200 | -0.430710 to 0.289804 |
| D/[log_4_2026-10-20-11-23-54.ulg](desktop_log_4_2026-10-20-11-23-54.json) | 19.293090 | OFFBOARD → ALTCTL → OFFBOARD | 449878727 | -2.511940 to 0.881213 | -1.079814 to 0.046884 | -0.796217 to 0.198182 |

All recorded local-position samples in all 12 files have `z_valid=1` and `v_z_valid=1`. This does not establish physical stability: z ranges differ substantially, and one recording shows resets and IMU clipping. No ULog dropout records are present in these files; absence of dropout records does not establish complete observation of every topic or all preceding/following activity.

Historical Offboard traces confirm PX4's logged Offboard state and heartbeat observations in those recordings. They do not identify the current companion build or validate the current transport. In `H/log_0_2026-10-20-10-02-06.ulg`, `D/log_0_2026-10-20-11-19-52.ulg`, and `D/log_4_2026-10-20-11-23-54.ulg`, the final return to `OFFBOARD` is recorded after disarming. The JSON includes arming states to keep those intervals distinct from armed operation.

## November 15 filename recording

Source: `/home/jikl62/桌面/log_10_2026-11-15-16-47-56.ulg`. SHA-256: `8d510d25fbea1e6473939765532ca051705f18cbf967a740fbe05f992490ee11`. ULog header start `62993146` us, final timestamp `70670498` us, duration `7.677352` s.

All 17 `vehicle_status` samples report Stabilized (`nav_state=15`), and `vehicle_control_mode.flag_control_offboard_enabled=0`. No Position or Offboard nav state and no `offboard_control_mode` samples are recorded. `failsafe=0` throughout the 17 status samples, while `failsafe_flags.offboard_control_signal_lost=1` in all 14 flag samples. Offboard signal lost is therefore not interchangeable with an active vehicle failsafe in this Stabilized window.

Timed log messages show initial Agent ping exhaustion at `65.835730` boot s (t=2.842584 s), DDS session setup failure at `66.836129` boot s (t=3.842983 s), and serial reinitialization of `/dev/ttyS3 @ 115200 baud` at `66.836591` boot s (t=3.843445 s). These support a DDS setup problem in this recording. They do not locate its root cause or prove physical UART silence. Untimed boot-console metadata includes earlier recordings and is preserved separately without assigning it to the current timeline.

The 76 local-position samples cover boot `63.264452` to `70.659846` s. Full-window local z changes from `+0.096944585` to `-1.485310912` m (span `1.582255498` m); vz ranges from `-0.519622624` to `+0.000270103` m/s and z_deriv from `-0.504988849` to `-0.002546937` m/s. The logged estimator indicates sustained upward motion in NED; actual ascent and estimation drift cannot be separated using this evidence alone.

The RC-disarm status transition occurs at boot `69.672397` s (t=6.679251 s). Before disarm, 66 local samples change z from `+0.096944585` to `-1.178031325` m. Ten local samples after disarm continue to `-1.485310912` m. The JSON includes separate local-position statistics by held arming state, so the full range is not presented as entirely armed flight.

`z_valid` and `v_z_valid` stay true, z/vz reset counters remain 1, and attitude reset counter remains 2. All horizontal, heading, and bottom-distance reset counters are also constant in this recording. `dist_bottom` reports values from about `0.09855` to `1.52384` m, but `dist_bottom_valid=0` in all 76 samples, so it does not supply a valid independent physical height reference. The `Takeoff detected` message at boot `64.132308` s is an onboard detector observation, not independent proof of liftoff or stable flight.

## Reset and event observations

| Recording | Observed reset-counter changes |
| --- | --- |
| H/log_0_2026-10-18-21-02-18_1.ulg | No observed counter changes |
| H/log_0_2026-10-20-10-02-06.ulg | No observed counter changes |
| H/log_0_2026-10-20-11-12-26_1.ulg | xy_reset_counter: 34 at t=0.268417s; 35 at t=1.965520s<br>vxy_reset_counter: 34 at t=0.268417s; 35 at t=1.965520s |
| H/log_2_2026-10-16-11-57-46_1.ulg | xy_reset_counter: 56 at t=0.298522s; 57 at t=4.701002s<br>z_reset_counter: 7 at t=0.298522s; 8 at t=4.701002s<br>vxy_reset_counter: 56 at t=0.298522s; 57 at t=4.701002s<br>vz_reset_counter: 7 at t=0.298522s; 8 at t=4.701002s<br>heading_reset_counter: 5 at t=0.298522s; 6 at t=4.701002s<br>dist_bottom_reset_counter: 7 at t=0.298522s; 8 at t=4.701002s<br>quat_reset_counter: 4 at t=0.297141s; 5 at t=4.700123s |
| H/log_4_2026-10-16-11-58-42_1.ulg | No observed counter changes |
| H/log_5_2026-10-16-12-00-18_1.ulg | No observed counter changes |
| D/log_0_2026-10-20-11-19-52.ulg | No observed counter changes |
| D/log_10_2026-11-15-16-47-56.ulg | No observed counter changes |
| D/log_1_2026-10-20-11-20-32.ulg | No observed counter changes |
| D/log_2_2026-10-20-11-21-14.ulg | No observed counter changes |
| D/log_3_2026-10-20-11-22-20.ulg | No observed counter changes |
| D/log_4_2026-10-20-11-23-54.ulg | No observed counter changes |

In `H/log_2_2026-10-16-11-57-46_1.ulg`, a logged primary-EKF filter-fault change at t=4.650457 s is followed by observed local and attitude counter changes around t=4.70 s. Its z span is about 12.75 m; IMU acceleration clipping messages are recorded. That recording cannot be treated as a smooth, stable height reference merely because its validity bits remain true. `H/log_4_2026-10-16-11-58-42_1.ulg` also records repeated acceleration clipping and a failsafe-warning event even though its sampled `vehicle_status.failsafe` remains false. The event stream and status boolean are both retained.

For older Offboard recordings, the final heartbeat, subsequent mode/failsafe events, kill actions, and disarm transitions are distinct observations. Their ordering alone does not identify whether companion withdrawal, scheduling, transport, PX4 input handling, operator action, or another mechanism caused the exit. The audit preserves every timed text log message, decoded event, and raw event, plus all `failsafe_flags` field transitions.

## Evidence and reproduction

- `inspect_ulogs.py`: full read-only parser. Uses installed pyulog and embedded event definitions only; its event fallback disables network retrieval.
- `audit.json`: complete machine-readable audit with source hashes, firmware, parameters, topic timing, all mode/arming/failsafe changes, local ranges/validity/reset counters, attitude reset transitions, final heartbeat, events, and parse results.
- `summary.json`: compact index with the principal fields for each source file.
- Per-recording `*.json`: full evidence for that source alone; each `*_local_position.csv` preserves the corresponding vertical sample sequence and validity/reset fields.
- `write_report.py`: regenerates this README and summary from `audit.json`.
- `SHA256SUMS`: content hashes for the generated inspection/report artifacts.

```bash
python3 /tmp/fc01-stage1-ulog-audit/inspect_ulogs.py
python3 /tmp/fc01-stage1-ulog-audit/write_report.py
```

An independent parser cross-checked all 12 source hashes, fully parsed durations, firmware hashes, observed mode sets, final heartbeat timestamps, z/vz/z_deriv extrema, vertical reset changes, and dropout counts with no discrepancy. See `/tmp/fc01-stage1-ulog-independent/REVIEW.md` and `independent_crosscheck.json`. Its review identified reset transitions that initially lacked interval filtering; the main script was corrected and regenerated, and interval summaries now filter counters to their own samples.

These artifacts establish historical PX4 internal observations and identify the missing Position evidence. They cannot establish a current stable Position height baseline, physical height accuracy, flight stability, current companion/PX4 pairing, current DDS operation, UART saturation, or the causes of prior failures. Those claims require a matching recording of the relevant mode and system identities, with a trustworthy independent height observation and an explicit stability acceptance criterion.
