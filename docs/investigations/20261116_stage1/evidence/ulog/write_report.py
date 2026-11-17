#!/usr/bin/env python3
"""Format the inspected evidence; does not open source ULogs."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
audit = json.loads((ROOT / "audit.json").read_text())
latest = next(r for r in audit["logs"] if Path(r["source"]).name == "log_10_2026-11-15-16-47-56.ulg")


def fmt_range(r, f):
    stats = r["local_position"][f]
    return f"{stats['min']:.6f} to {stats['max']:.6f}"


def reset_text(r):
    changes = []
    for f, v in r["local_position"].items():
        if "reset_counter" in f and v["observed_change_count"]:
            changes.append(f"{f}: " + "; ".join(f"{x['value']} at t={x['seconds_since_ulog_header']:.6f}s" for x in v["changes"]))
    for d in r["attitude"]:
        v = d["quat_reset_counter"]
        if v["observed_change_count"]:
            changes.append("quat_reset_counter: " + "; ".join(f"{x['value']} at t={x['seconds_since_ulog_header']:.6f}s" for x in v["changes"]))
    return "<br>".join(changes) or "No observed counter changes"


lines = ["# FC01 stage 1: read-only ULog evidence, 2026-11-16", "",
         "**No Position baseline is demonstrated.** All 12 directly available candidate ULogs parse successfully, and none contains `vehicle_status.nav_state=2` (Position). Six older recordings contain both `nav_state=14` (Offboard) and logged `offboard_control_mode` samples. The November 15 filename recording contains only `nav_state=15` (Stabilized), with DDS session failure and no recorded Offboard heartbeat.", "",
         "The audit treats every file as an independent recording. It does not correlate November 14 companion logs with the November 15 filename PX4 recording. No repo files, source ULogs, hardware, services, serial ports, SSH sessions, or flight parameters were changed.", "",
         "## Scope and method", "",
         "The source set is exactly `*.ulg` directly inside `/home/jikl62` and `/home/jikl62/桌面`; archives and deeper directories were excluded. Actual recorded `nav_state` determines classification. Filename dates are labels, not independently verified time or flight identity. All 12 firmware hashes are `07983bfb894b9372d7520d43ac10dc792d0b4b9f`, and hardware reports `DAMIAO_DM_FC01`; a common firmware hash does not establish that the same parameters, companion binary, sensors, or physical setup were used.", "",
         "Mode mapping: `1=ALTCTL`, `2=POSCTL`, `14=OFFBOARD`, `15=STAB`. Arming states: `1=disarmed`, `2=armed`. ULog timestamps are PX4 boot-relative microseconds. Relative t is measured from the ULog header start; some buffered status/events precede that header and intentionally retain negative t. A mode interval ends at the next recorded transition or the last timestamp in the fully parsed ULog. Logged sample frequency is not UART throughput or actual companion publication frequency.", "",
         "Local z/vz/z_deriv use NED coordinates: downward is positive; z is relative to a local estimator origin and is not physical height above ground. `z_valid` and `v_z_valid` are estimator validity flags. Counter changes identify observed estimator reference resets, and constant counters only establish no observed change within the logged window.", "",
         "## Per-recording classification", "",
         "`H` means `/home/jikl62/`; `D` means `/home/jikl62/桌面/`. Duration is ULog header start to the last timestamp after parsing all topics. Last heartbeat is the final logged `offboard_control_mode.timestamp`, in PX4 boot microseconds. Full nav and arming transitions with timestamped ranges are in each JSON.", "",
         "| Source | Duration s | Recorded nav transitions | Last heartbeat us | z range m | vz range m/s | z_deriv range m/s |",
         "| --- | ---: | --- | ---: | --- | --- | --- |"]
for r in audit["logs"]:
    prefix = "H" if Path(r["source"]).parent == Path("/home/jikl62") else "D"
    name = Path(r["source"]).name
    mode = " → ".join(s["name"] for s in r["nav_segments"])
    heartbeat = r["offboard_heartbeat"].get("last_timestamp_us") or "absent"
    lines.append(f"| {prefix}/[{name}]({r['evidence_json']}) | {r['ulog_duration_s']:.6f} | {mode} | {heartbeat} | {fmt_range(r,'z')} | {fmt_range(r,'vz')} | {fmt_range(r,'z_deriv')} |")
lines += ["", "All recorded local-position samples in all 12 files have `z_valid=1` and `v_z_valid=1`. This does not establish physical stability: z ranges differ substantially, and one recording shows resets and IMU clipping. No ULog dropout records are present in these files; absence of dropout records does not establish complete observation of every topic or all preceding/following activity.", "",
          "Historical Offboard traces confirm PX4's logged Offboard state and heartbeat observations in those recordings. They do not identify the current companion build or validate the current transport. In `H/log_0_2026-10-20-10-02-06.ulg`, `D/log_0_2026-10-20-11-19-52.ulg`, and `D/log_4_2026-10-20-11-23-54.ulg`, the final return to `OFFBOARD` is recorded after disarming. The JSON includes arming states to keep those intervals distinct from armed operation.", "",
          "## November 15 filename recording", "",
          f"Source: `{latest['source']}`. SHA-256: `{latest['sha256']}`. ULog header start `{latest['ulog_header_start_timestamp_us']}` us, final timestamp `{latest['ulog_last_timestamp_us']}` us, duration `{latest['ulog_duration_s']:.6f}` s.", "",
          "All 17 `vehicle_status` samples report Stabilized (`nav_state=15`), and `vehicle_control_mode.flag_control_offboard_enabled=0`. No Position or Offboard nav state and no `offboard_control_mode` samples are recorded. `failsafe=0` throughout the 17 status samples, while `failsafe_flags.offboard_control_signal_lost=1` in all 14 flag samples. Offboard signal lost is therefore not interchangeable with an active vehicle failsafe in this Stabilized window.", "",
          "Timed log messages show initial Agent ping exhaustion at `65.835730` boot s (t=2.842584 s), DDS session setup failure at `66.836129` boot s (t=3.842983 s), and serial reinitialization of `/dev/ttyS3 @ 115200 baud` at `66.836591` boot s (t=3.843445 s). These support a DDS setup problem in this recording. They do not locate its root cause or prove physical UART silence. Untimed boot-console metadata includes earlier recordings and is preserved separately without assigning it to the current timeline.", "",
          "The 76 local-position samples cover boot `63.264452` to `70.659846` s. Full-window local z changes from `+0.096944585` to `-1.485310912` m (span `1.582255498` m); vz ranges from `-0.519622624` to `+0.000270103` m/s and z_deriv from `-0.504988849` to `-0.002546937` m/s. The logged estimator indicates sustained upward motion in NED; actual ascent and estimation drift cannot be separated using this evidence alone.", "",
          "The RC-disarm status transition occurs at boot `69.672397` s (t=6.679251 s). Before disarm, 66 local samples change z from `+0.096944585` to `-1.178031325` m. Ten local samples after disarm continue to `-1.485310912` m. The JSON includes separate local-position statistics by held arming state, so the full range is not presented as entirely armed flight.", "",
          "`z_valid` and `v_z_valid` stay true, z/vz reset counters remain 1, and attitude reset counter remains 2. All horizontal, heading, and bottom-distance reset counters are also constant in this recording. `dist_bottom` reports values from about `0.09855` to `1.52384` m, but `dist_bottom_valid=0` in all 76 samples, so it does not supply a valid independent physical height reference. The `Takeoff detected` message at boot `64.132308` s is an onboard detector observation, not independent proof of liftoff or stable flight.", "",
          "## Reset and event observations", "",
          "| Recording | Observed reset-counter changes |",
          "| --- | --- |"]
for r in audit["logs"]:
    prefix = "H" if Path(r["source"]).parent == Path("/home/jikl62") else "D"
    lines.append(f"| {prefix}/{Path(r['source']).name} | {reset_text(r)} |")
lines += ["", "In `H/log_2_2026-10-16-11-57-46_1.ulg`, a logged primary-EKF filter-fault change at t=4.650457 s is followed by observed local and attitude counter changes around t=4.70 s. Its z span is about 12.75 m; IMU acceleration clipping messages are recorded. That recording cannot be treated as a smooth, stable height reference merely because its validity bits remain true. `H/log_4_2026-10-16-11-58-42_1.ulg` also records repeated acceleration clipping and a failsafe-warning event even though its sampled `vehicle_status.failsafe` remains false. The event stream and status boolean are both retained.", "",
          "For older Offboard recordings, the final heartbeat, subsequent mode/failsafe events, kill actions, and disarm transitions are distinct observations. Their ordering alone does not identify whether companion withdrawal, scheduling, transport, PX4 input handling, operator action, or another mechanism caused the exit. The audit preserves every timed text log message, decoded event, and raw event, plus all `failsafe_flags` field transitions.", "",
          "## Evidence and reproduction", "",
          "- `inspect_ulogs.py`: full read-only parser. Uses installed pyulog and embedded event definitions only; its event fallback disables network retrieval.",
          "- `audit.json`: complete machine-readable audit with source hashes, firmware, parameters, topic timing, all mode/arming/failsafe changes, local ranges/validity/reset counters, attitude reset transitions, final heartbeat, events, and parse results.",
          "- `summary.json`: compact index with the principal fields for each source file.",
          "- Per-recording `*.json`: full evidence for that source alone; each `*_local_position.csv` preserves the corresponding vertical sample sequence and validity/reset fields.",
          "- `write_report.py`: regenerates this README and summary from `audit.json`.",
          "- `SHA256SUMS`: content hashes for the generated inspection/report artifacts.", "",
          "```bash\npython3 /tmp/fc01-stage1-ulog-audit/inspect_ulogs.py\npython3 /tmp/fc01-stage1-ulog-audit/write_report.py\n```", "",
          "An independent parser cross-checked all 12 source hashes, fully parsed durations, firmware hashes, observed mode sets, final heartbeat timestamps, z/vz/z_deriv extrema, vertical reset changes, and dropout counts with no discrepancy. See `/tmp/fc01-stage1-ulog-independent/REVIEW.md` and `independent_crosscheck.json`. Its review identified reset transitions that initially lacked interval filtering; the main script was corrected and regenerated, and interval summaries now filter counters to their own samples.", "",
          "These artifacts establish historical PX4 internal observations and identify the missing Position evidence. They cannot establish a current stable Position height baseline, physical height accuracy, flight stability, current companion/PX4 pairing, current DDS operation, UART saturation, or the causes of prior failures. Those claims require a matching recording of the relevant mode and system identities, with a trustworthy independent height observation and an explicit stability acceptance criterion.", ""]
(ROOT / "README.md").write_text("\n".join(lines))

summary = {k: audit[k] for k in ("audit_generated_utc", "scope", "counts", "parse_errors")}
summary["logs"] = [{k: r[k] for k in ("source", "sha256", "size_bytes", "ulog_duration_s", "evidence_json",
                                      "firmware_and_hardware", "classification", "offboard_heartbeat")}
                   | {"nav_segments": [{k: s[k] for k in ("nav_state", "name", "start", "end_exclusive_or_ulog_last", "duration_s", "arming_states")} for s in r["nav_segments"]],
                      "local_vertical": {f: r["local_position"][f] for f in ("z", "vz", "z_deriv", "z_valid", "v_z_valid", "z_reset_counter", "vz_reset_counter")},
                      "attitude_reset": [d["quat_reset_counter"] for d in r["attitude"]],
                      "status_failsafe": r["vehicle_status"]["failsafe"]}
                   for r in audit["logs"]]
(ROOT / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
files = sorted(p for p in ROOT.iterdir() if p.is_file() and p.name != "SHA256SUMS")
(ROOT / "SHA256SUMS").write_text("".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n" for p in files))
print(ROOT / "README.md")
