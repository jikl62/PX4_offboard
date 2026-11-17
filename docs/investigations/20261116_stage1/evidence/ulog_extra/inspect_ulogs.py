#!/usr/bin/env python3
"""Read local ULogs without ROS, hardware access, flight alignment, or network."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from pyulog import ULog
from pyulog.px4_events import PX4Events

NAV_NAMES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
             4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "POSITION_SLOW",
             10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
             15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
             19: "AUTO_FOLLOW_TARGET", 20: "AUTO_PRECLAND", 21: "ORBIT"}
PARAM_PREFIXES = ("UXRCE_", "COM_OF_", "COM_OBL_", "COM_RC_", "EKF2_HGT_",
                  "EKF2_BARO_", "EKF2_RNG_", "EKF2_EV_", "MPC_THR_",
                  "MAV_", "SDLOG_")
PARAM_KEYS = {"COM_POS_FS_DELAY", "COM_POS_LOW_ACT", "SYS_AUTOSTART",
              "EKF2_HGT_REF", "EKF2_EV_CTRL", "EKF2_RNG_CTRL"}
VALIDITY_KEYS = ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid",
                 "heading_good_for_control", "xy_global", "z_global",
                 "dist_bottom_valid", "dead_reckoning")


def basic(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, dict):
        return {str(k): basic(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [basic(v) for v in value]
    return value


def numeric(values):
    a = np.asarray(values, dtype=np.float64)
    good = a[np.isfinite(a)]
    result = {"samples": len(a), "finite_samples": len(good),
              "nonfinite_samples": int(len(a) - len(good))}
    if len(good):
        result.update(min=float(np.min(good)), max=float(np.max(good)),
                      span=float(np.ptp(good)), mean=float(np.mean(good)),
                      std=float(np.std(good)), median=float(np.median(good)),
                      p05=float(np.percentile(good, 5)), p95=float(np.percentile(good, 95)),
                      first=float(a[0]) if np.isfinite(a[0]) else None,
                      last=float(a[-1]) if np.isfinite(a[-1]) else None)
    return result


def time_record(t, start):
    return {"timestamp_us": int(t), "seconds_since_ulog_header": (int(t) - int(start)) / 1e6}


def timing(ds, start):
    ts = np.asarray(ds.data["timestamp"], dtype=np.int64)
    result = {"topic": ds.name, "instance": int(ds.multi_id), "samples": len(ts)}
    if len(ts):
        result.update(first=time_record(ts[0], start), last=time_record(ts[-1], start),
                      timestamp_span_s=(int(ts[-1]) - int(ts[0])) / 1e6)
    if len(ts) > 1:
        dt = np.diff(ts).astype(float) / 1e6
        result["sample_intervals_s"] = numeric(dt)
        result["nonpositive_intervals"] = int(np.count_nonzero(dt <= 0))
        result["observed_logger_samples_per_s"] = (len(ts) - 1) / float(np.sum(dt)) if np.sum(dt) > 0 else None
    if "timestamp_sample" in ds.data:
        age = (ts - np.asarray(ds.data["timestamp_sample"], dtype=np.int64)) / 1e6
        result["publication_minus_sample_s"] = numeric(age)
    return result


def transitions(ds, field, start, mask=None):
    if field not in ds.data:
        return {"present": False}
    original_indexes = np.arange(len(ds.data[field]))
    if mask is not None:
        original_indexes = original_indexes[mask]
    values = ds.data[field][original_indexes]
    counts = [{"value": basic(v), "samples": int(np.count_nonzero(values == v))}
              for v in np.unique(values)]
    indexes = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1] if len(values) else []
    changes = []
    for i in indexes:
        original_index = int(original_indexes[i])
        row = time_record(ds.data["timestamp"][original_index], start)
        row.update(sample_index=original_index, value=basic(values[i]), initial=bool(i == 0))
        changes.append(row)
    return {"present": True, "counts": counts, "changes": changes,
            "observed_change_count": max(0, len(changes) - 1)}


def local_summary(ds, start, mask=None):
    result = timing(ds, start) if mask is None else {"samples": int(np.count_nonzero(mask))}
    if mask is None:
        mask = np.ones(len(ds.data["timestamp"]), dtype=bool)
    for field in ("x", "y", "z", "vx", "vy", "vz", "z_deriv", "dist_bottom", "epv", "evv"):
        if field in ds.data:
            result[field] = numeric(ds.data[field][mask])
    for field in VALIDITY_KEYS:
        if field in ds.data:
            result[field] = transitions(ds, field, start) if np.all(mask) else {
                "counts": [{"value": basic(v), "samples": int(np.count_nonzero(ds.data[field][mask] == v))}
                           for v in np.unique(ds.data[field][mask])]}
    for field in ds.data:
        if "reset_counter" in field:
            result[field] = transitions(ds, field, start, mask)
    for field in ("delta_z", "delta_vz", "delta_heading", "delta_dist_bottom"):
        if field in ds.data:
            result[field] = numeric(ds.data[field][mask])
    if "vz" in ds.data and "z_deriv" in ds.data:
        result["vz_minus_z_deriv"] = numeric(ds.data["vz"][mask] - ds.data["z_deriv"][mask])
    return result


def nav_segments(status, lp, end, start):
    values = status.data["nav_state"]
    ts = status.data["timestamp"].astype(np.int64)
    starts = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1] if len(values) else []
    result = []
    for k, idx in enumerate(starts):
        stop_idx = int(starts[k + 1]) if k + 1 < len(starts) else len(values)
        stop = int(ts[stop_idx]) if stop_idx < len(ts) else int(end)
        row = {"nav_state": int(values[idx]), "name": NAV_NAMES.get(int(values[idx]), "UNKNOWN"),
               "start": time_record(ts[idx], start), "end_exclusive_or_ulog_last": time_record(stop, start),
               "duration_s": (stop - int(ts[idx])) / 1e6,
               "vehicle_status_samples": stop_idx - int(idx),
               "arming_states": sorted(set(int(v) for v in status.data.get("arming_state", [])[idx:stop_idx]))}
        if lp is not None:
            lts = lp.data["timestamp"]
            mask = (lts >= ts[idx]) & ((lts < stop) if stop_idx < len(ts) else (lts <= stop))
            row["local_position_during_interval"] = local_summary(lp, start, mask)
        result.append(row)
    return result


def scan(path, output_dir):
    before = path.stat()
    raw = path.read_bytes()
    u = ULog(str(path))
    start, end = int(u.start_timestamp), int(u.last_timestamp)
    datasets = {(d.name, int(d.multi_id)): d for d in u.data_list}
    status = datasets.get(("vehicle_status", 0))
    lp = datasets.get(("vehicle_local_position", 0))
    off = datasets.get(("offboard_control_mode", 0))
    result = {"source": str(path), "filename_is_not_mode_or_flight_identity": True,
              "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw),
              "source_mtime_utc": datetime.fromtimestamp(before.st_mtime, timezone.utc).isoformat(),
              "ulog_header_start_timestamp_us": start, "ulog_last_timestamp_us": end,
              "ulog_duration_s": (end - start) / 1e6,
              "firmware_and_hardware": basic(u.msg_info_dict),
              "time_domain": "ULog/PX4 boot-relative microseconds; filename date not independently validated; no companion-time alignment",
              "dropouts": [{"timestamp_us": int(d.timestamp), "duration_ms": int(d.duration)} for d in u.dropouts],
              "topics": [timing(d, start) for d in u.data_list],
              "selected_initial_parameters": {k: basic(v) for k, v in u.initial_parameters.items()
                                              if k in PARAM_KEYS or k.startswith(PARAM_PREFIXES)},
              "changed_parameters": basic(u.changed_parameters)}
    if status is not None:
        result["vehicle_status"] = timing(status, start)
        for key in ("nav_state", "nav_state_user_intention", "arming_state", "failsafe",
                    "failsafe_and_user_took_over", "failure_detector_status", "pre_flight_checks_pass",
                    "latest_arming_reason", "latest_disarming_reason"):
            result["vehicle_status"][key] = transitions(status, key, start)
        result["nav_segments"] = nav_segments(status, lp, end, start)
    else:
        result["vehicle_status"] = {"present": False}
        result["nav_segments"] = []
    result["local_position"] = local_summary(lp, start) if lp is not None else {"present": False}
    result["local_position_by_arming_state"] = []
    if status is not None and lp is not None and "arming_state" in status.data:
        sts = status.data["timestamp"].astype(np.int64)
        lts = lp.data["timestamp"].astype(np.int64)
        previous = np.searchsorted(sts, lts, side="right") - 1
        known = previous >= 0
        held_state = np.zeros(len(lts), dtype=np.int64)
        held_state[known] = status.data["arming_state"][previous[known]]
        for state in sorted(set(int(v) for v in held_state[known])):
            mask = known & (held_state == state)
            row = {"arming_state": state, "method": "Most recent logged vehicle_status at each local-position timestamp",
                   "local_position": local_summary(lp, start, mask)}
            row["first_local_sample"] = time_record(lts[mask][0], start)
            row["last_local_sample"] = time_record(lts[mask][-1], start)
            result["local_position_by_arming_state"].append(row)
    attitudes = [d for d in u.data_list if d.name == "vehicle_attitude"]
    result["attitude"] = []
    for d in attitudes:
        row = timing(d, start)
        row["quat_reset_counter"] = transitions(d, "quat_reset_counter", start)
        row["delta_q_reset"] = {key: numeric(v) for key, v in d.data.items() if key.startswith("delta_q_reset")}
        result["attitude"].append(row)
    result["offboard_heartbeat"] = {"topic": "offboard_control_mode", "present": off is not None,
                                      "last_timestamp_us": None, "last_seconds_since_ulog_header": None}
    if off is not None:
        result["offboard_heartbeat"].update(timing(off, start))
        if len(off.data["timestamp"]):
            last = int(off.data["timestamp"][-1])
            result["offboard_heartbeat"].update(last_timestamp_us=last,
                                                last_seconds_since_ulog_header=(last - start) / 1e6,
                                                ulog_end_minus_last_heartbeat_s=(end - last) / 1e6)
        result["offboard_heartbeat"]["fields"] = {key: transitions(off, key, start)
                                                    for key in off.data if key != "timestamp"}
    else:
        result["offboard_heartbeat"]["absence_limit"] = "No samples in this ULog; alone this cannot establish unpublished topics were logged or prove physical link silence."
    result["failsafe_flags"] = []
    for d in u.data_list:
        if d.name == "failsafe_flags":
            row = timing(d, start)
            row["fields"] = {key: transitions(d, key, start) for key in d.data if key != "timestamp"}
            result["failsafe_flags"].append(row)
    result["vehicle_control_mode"] = []
    for d in u.data_list:
        if d.name == "vehicle_control_mode":
            row = timing(d, start)
            row["fields"] = {key: transitions(d, key, start) for key in d.data if key.startswith("flag_")}
            result["vehicle_control_mode"].append(row)
    result["land_detection"] = []
    for d in u.data_list:
        if d.name == "vehicle_land_detected":
            row = timing(d, start)
            row["fields"] = {key: transitions(d, key, start) for key in d.data if key != "timestamp"}
            result["land_detection"].append(row)
    result["logged_messages"] = [dict(time_record(m.timestamp, start), level_ascii=int(m.log_level),
                                       severity=int(m.log_level) - 48, message=m.message)
                                    for m in u.logged_messages]
    event_parser = PX4Events()
    event_parser.set_default_json_definitions_cb(lambda _: None)  # Embedded metadata only, no network.
    result["decoded_events"] = [dict(time_record(t, start), level=level, message=msg)
                                  for t, level, msg in event_parser.get_logged_events(u)]
    result["raw_events"] = []
    for d in u.data_list:
        if d.name == "event":
            for i in range(len(d.data["timestamp"])):
                result["raw_events"].append({key: basic(v[i]) for key, v in d.data.items()})
    boot = []
    for chunks in u.msg_info_multiple_dict.get("boot_console_output", []):
        boot.append("".join(chunk.decode(errors="replace") if isinstance(chunk, bytes) else chunk for chunk in chunks))
    result["boot_console_output"] = boot
    result["boot_console_time_limit"] = "Untimed metadata can include activity before this ULog; do not place its messages on the data timeline."
    result["console_dds_failure_lines"] = [line for text in boot for line in text.splitlines()
                                           if "uxrce_dds_client" in line or "timesync" in line]
    modes = sorted(set(int(x) for x in status.data["nav_state"])) if status is not None else []
    result["classification"] = {"observed_nav_states": modes,
                                "position_nav_state_2_observed": 2 in modes,
                                "offboard_nav_state_14_observed": 14 in modes,
                                "offboard_heartbeat_samples_observed": off is not None and len(off.data["timestamp"]) > 0,
                                "position_stable_height_baseline_demonstrated": False,
                                "stable_height_limit": "No numerical physical-height acceptance threshold or independent height measurement supplied; report ranges, do not infer flight stability from estimator validity."}
    after = path.stat()
    result["source_unchanged_during_inspection"] = before.st_size == after.st_size and before.st_mtime_ns == after.st_mtime_ns
    safe_name = path.name.removesuffix(".ulg")
    if path.parent == Path("/home/jikl62"):
        safe_name = "home_" + safe_name
    else:
        safe_name = "desktop_" + safe_name
    relative = f"{safe_name}.json"
    (output_dir / relative).write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    result["evidence_json"] = relative
    if lp is not None:
        fields = ["timestamp", "timestamp_sample", "z", "vz", "z_deriv", "z_valid", "v_z_valid",
                  "z_reset_counter", "vz_reset_counter", "xy_reset_counter", "vxy_reset_counter",
                  "heading_reset_counter", "dist_bottom", "dist_bottom_valid"]
        fields = [f for f in fields if f in lp.data]
        csv_name = f"{safe_name}_local_position.csv"
        with (output_dir / csv_name).open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(fields)
            for i in range(len(lp.data["timestamp"])):
                writer.writerow([basic(lp.data[field][i]) for field in fields])
        result["local_position_csv"] = csv_name
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    paths = sorted({p for root in (Path("/home/jikl62"), Path("/home/jikl62/桌面")) for p in root.glob("*.ulg")})
    results, errors = [], []
    for path in paths:
        try:
            results.append(scan(path, args.output))
        except Exception as exc:
            errors.append({"source": str(path), "error": repr(exc)})
    audit = {"audit_generated_utc": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
             "pyulog": importlib.metadata.version("pyulog"), "numpy": np.__version__,
             "scope": "Only *.ulg directly inside /home/jikl62 and /home/jikl62/桌面; no archives, hardware, ROS, SSH or companion alignment",
             "mode_mapping": NAV_NAMES, "logs": results, "parse_errors": errors,
             "counts": {"files_discovered": len(paths), "parsed": len(results), "errors": len(errors),
                        "with_position_nav2": sum(r["classification"]["position_nav_state_2_observed"] for r in results),
                        "with_offboard_nav14": sum(r["classification"]["offboard_nav_state_14_observed"] for r in results),
                        "with_offboard_heartbeat": sum(r["classification"]["offboard_heartbeat_samples_observed"] for r in results)}}
    (args.output / "audit.json").write_text(json.dumps(audit, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps(audit["counts"]))


if __name__ == "__main__":
    main()
