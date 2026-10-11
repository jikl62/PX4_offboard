#!/usr/bin/env python3
"""Inspect local configuration/captured logs. No ROS, ports or command writes."""
import argparse
import json
import math
from pathlib import Path
import yaml

GATES=('flight_actions_enabled','arm_enabled','takeoff_enabled','flight_parameters_confirmed')
def audit_parameters(parameters):
    closed=all(parameters.get(k) is False for k in GATES)
    fallback=parameters.get('px4_land_fallback_confirmed') is True
    return {'closed_gate_check_pass':closed,'four_gate_values':{k:parameters.get(k) for k in GATES},
            'px4_land_contract_configured':fallback,
            'external_fallback_execution_verified':False,
            'powered_flight_go':False,
            'scope':'LOCAL_CONFIG_CHECK_ONLY',
            'remaining':['C_R6_INDEPENDENT_REVIEW','NATIVE_HUMBLE_BUILD',
                         'ACTUAL_FC01_FALLBACK_WITH_CURRENT_SENSORS','ACTUAL_AIRFRAME_CALIBRATION']}

def check_config(path):
    document=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    parameters=document['fc01_system']['ros__parameters']
    if not isinstance(parameters,dict):raise ValueError('invalid ROS parameters')
    return audit_parameters(parameters)

def check_events(rows):
    events=[r for r in rows if r.get('kind')=='phase_event']
    partial=0;missing=0;exceptions=0;last_tick=None;last_mode=None;max_gap=0.;ordered=True;commits=0
    for row in events:
        stamp=row.get('monotonic_s')
        if isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp):
            ordered=False;continue
        if last_tick is not None and stamp<=last_tick:ordered=False
        last_tick=stamp
        target=row.get('target_local_commit') is True;mode=row.get('heartbeat_local_commit') is True
        if row.get('sink_exception') is True:exceptions+=1
        if target!=mode:partial+=1
        if row.get('armed') is True and row.get('nav_state')==14 and (not target or not mode):missing+=1
        if mode:
            commits+=1
            if last_mode is not None:max_gap=max(max_gap,stamp-last_mode)
            last_mode=stamp
        gap=row.get('max_heartbeat_gap_s',0)
        if isinstance(gap,(int,float)) and math.isfinite(gap):max_gap=max(max_gap,gap)
    if last_tick is not None and last_mode is not None:max_gap=max(max_gap,last_tick-last_mode)
    return {'phase_event_rows':len(events),'heartbeat_local_commits':commits,'partial_sink_ticks':partial,
            'max_local_heartbeat_gap_s':max_gap,'monotonic_order_valid':ordered,'armed_offboard_missing_sink_ticks':missing,
            'sink_exception_ticks':exceptions,
            'local_dual_sink_check_pass':bool(events) and commits>0 and ordered and partial==0 and missing==0 and exceptions==0 and max_gap<=.3,
            'px4_receipt_proven':False,'fallback_execution_verified':False}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--events',type=Path)
    args=parser.parse_args()
    try:
        result=check_config(args.config)
        if args.events:
            rows=[json.loads(line) for line in args.events.read_text(encoding='utf-8').splitlines() if line.strip().startswith('{')]
            result['captured_event_check']=check_events(rows)
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
        ok=result['closed_gate_check_pass']
        if args.events:ok=ok and result['captured_event_check']['local_dual_sink_check_pass']
        return 0 if ok else 1
    except (ValueError,KeyError,OSError,TypeError,yaml.YAMLError) as error:
        print(json.dumps({'closed_gate_check_pass':False,'powered_flight_go':False,'error':str(error)}))
        return 2
if __name__=='__main__':raise SystemExit(main())
