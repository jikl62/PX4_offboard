#!/usr/bin/env python3
"""Independent read-only cross-check of direct home and desktop ULogs."""
from pathlib import Path
from hashlib import sha256
import json
import numpy as np
from pyulog import ULog

NAV = {1: "Altitude", 2: "Position", 14: "Offboard", 15: "Stabilized"}
def native(x):
    if isinstance(x, np.generic): return x.item()
    raise TypeError(type(x).__name__)
def transitions(data, fields):
    result=[]
    previous=None
    for i,t in enumerate(data["timestamp"]):
        value=tuple(native(data[k][i]) for k in fields)
        if value != previous:
            result.append({"timestamp_us": int(t), **dict(zip(fields, value))})
        previous=value
    return result
def span(data, key, mask=None):
    value=data[key] if mask is None else data[key][mask]
    finite=value[np.isfinite(value)]
    return {"samples":len(value), "finite_samples":len(finite), "min":float(np.min(finite)) if len(finite) else None, "max":float(np.max(finite)) if len(finite) else None, "first":native(value[0]) if len(value) else None, "last":native(value[-1]) if len(value) else None}

def inspect(p):
    u=ULog(str(p))
    topics={d.name:d.data for d in u.data_list if d.multi_id==0}
    status=topics["vehicle_status"]
    position=topics["vehicle_local_position"]
    index=np.searchsorted(status["timestamp"], position["timestamp"], side="right")-1
    # Samples before first logged status are not classified by a future status.
    aligned_nav=np.full(len(index),-1, dtype=int)
    aligned_armed=np.zeros(len(index),dtype=bool)
    known=index >= 0
    aligned_nav[known]=status["nav_state"][index[known]]
    aligned_armed[known]=status["arming_state"][index[known]] == 2
    heartbeat=topics.get("offboard_control_mode")
    attitude=topics.get("vehicle_attitude")
    fields=[k for k in ("z", "vz", "z_deriv", "z_valid", "v_z_valid", "z_reset_counter", "vz_reset_counter") if k in position]
    return {
      "path":str(p), "bytes":p.stat().st_size, "sha256":sha256(p.read_bytes()).hexdigest(),
      "firmware_hash":u.msg_info_dict.get("ver_sw"),
      "header_start_us":u.start_timestamp, "last_timestamp_us":u.last_timestamp,
      "header_duration_s":(u.last_timestamp-u.start_timestamp)/1e6,
      "nav_states": [{"value":int(x), "name":NAV.get(int(x),"other")} for x in np.unique(status["nav_state"])],
      "status_transitions":transitions(status,["nav_state","arming_state","failsafe"]),
      "local_position_all":{k:span(position,k) for k in fields},
      "local_position_armed_by_nav":{str(int(nav)): {k:span(position,k, (aligned_nav==nav)&aligned_armed) for k in fields} for nav in np.unique(aligned_nav[aligned_armed])},
      "vertical_reset_transitions":transitions(position,["z_reset_counter","vz_reset_counter"]),
      "attitude_reset_transitions":transitions(attitude,["quat_reset_counter"]) if attitude and "quat_reset_counter" in attitude else None,
      "offboard_control_mode": {"present":heartbeat is not None,"samples":len(heartbeat["timestamp"]) if heartbeat else 0,"first_us":int(heartbeat["timestamp"][0]) if heartbeat else None,"last_us":int(heartbeat["timestamp"][-1]) if heartbeat else None,"max_logged_interval_s":float(np.max(np.diff(heartbeat["timestamp"]))/1e6) if heartbeat and len(heartbeat["timestamp"])>1 else None},
      "logged_messages":[{"timestamp_us":m.timestamp,"level":m.log_level,"message":m.message} for m in u.logged_messages],
      "dropouts":[{"timestamp_us":d.timestamp,"duration_ms":d.duration} for d in u.dropouts],
      "event_samples":len(topics.get("event",{}).get("timestamp",[])),
      "missing_topics": [name for name in ["vehicle_visual_odometry", "vehicle_odometry", "offboard_control_mode", "trajectory_setpoint"] if name not in topics],
    }

if __name__=="__main__":
    paths=sorted([*Path("/home/jikl62").glob("*.ulg"),*Path("/home/jikl62/桌面").glob("*.ulg")])
    result={"scope":"Only *.ulg directly in home and desktop; each log treated independently", "logs":[inspect(p) for p in paths]}
    result["counts"]={"logs":len(paths),"position_logs":sum(any(x["value"]==2 for x in l["nav_states"]) for l in result["logs"]),"offboard_logs":sum(any(x["value"]==14 for x in l["nav_states"]) for l in result["logs"])}
    output=Path(__file__).with_name("independent_crosscheck.json")
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    print(output)
