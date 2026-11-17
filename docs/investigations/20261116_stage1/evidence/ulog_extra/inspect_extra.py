#!/usr/bin/env python3
"""Inspect one archived simulator Position interval; no hardware or network."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog

OUT = Path(__file__).resolve().parent
REPO = Path('/home/jikl62/FC01_OrangePi_Navigation_20261028')
SOURCE = Path('/home/jikl62/DroneProjects/05_飞行日志与测试证据/原下载目录/FC01_M1_S10_failure.ulg')
SPEC = importlib.util.spec_from_file_location('stage1_inspect', OUT/'inspect_ulogs.py')
helper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(helper)
OUT.mkdir(exist_ok=True)
full = helper.scan(SOURCE, OUT)
u = ULog(str(SOURCE))
status = u.get_dataset('vehicle_status')
position = [s for s in full['nav_segments'] if s['nav_state'] == 2]
assert len(position) == 1
segment = position[0]
lo, hi = segment['start']['timestamp_us'], segment['end_exclusive_or_ulog_last']['timestamp_us']


def at_interval(ds):
    ts = np.asarray(ds.data['timestamp'], dtype=np.int64)
    mask = (ts >= lo) & (ts < hi)
    previous = np.flatnonzero(ts < lo)
    last_before = int(previous[-1]) if len(previous) else None
    result = {'topic': ds.name, 'instance': int(ds.multi_id), 'interval_samples': int(np.count_nonzero(mask)),
              'boundary': '[start_us, end_us), end is transition out of Position',
              'last_sample_before_interval': None, 'fields': {}}
    if last_before is not None:
        result['last_sample_before_interval'] = {
            key: helper.basic(v[last_before]) for key, v in ds.data.items()}
    indexes = np.flatnonzero(mask)
    result['timestamps_us'] = [int(ts[i]) for i in indexes]
    for field, values in ds.data.items():
        if field == 'timestamp':
            continue
        binary_or_counter = (field.startswith(('cs_', 'fs_', 'reject_', 'flag_')) or
                             'reset_counter' in field or 'reset_count' in field or
                             field in ('landed', 'ground_contact', 'maybe_landed', 'at_rest',
                                       'nav_state', 'arming_state', 'failsafe', 'fused',
                                       'innovation_rejected', 'z_valid', 'v_z_valid', 'dist_bottom_valid',
                                       'battery_warning', 'offboard_control_signal_lost'))
        if binary_or_counter:
            result['fields'][field] = helper.transitions(ds, field, u.start_timestamp, mask)
        else:
            result['fields'][field] = helper.numeric(values[mask])
    return result


def finite_json(value):
    value = helper.basic(value)
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


topic_names = {'vehicle_status', 'vehicle_control_mode', 'vehicle_land_detected',
               'vehicle_attitude', 'vehicle_local_position', 'vehicle_local_position_setpoint',
               'failsafe_flags', 'estimator_status', 'estimator_status_flags'}
topic_names.update(d.name for d in u.data_list if d.name.startswith(('estimator_aid_src_', 'distance_sensor', 'sensor_optical_flow', 'vehicle_optical_flow', 'vehicle_visual_odometry')))
details = [at_interval(d) for d in u.data_list if d.name in topic_names]
all_params = helper.basic(u.initial_parameters)
fusion_params = {k:v for k,v in all_params.items() if k.startswith(('EKF2_', 'SENS_', 'UXRCE_'))
                 or k in ('SYS_AUTOSTART', 'SYS_HAS_GPS', 'SYS_HAS_BARO', 'SYS_HAS_NUM_DIST', 'SYS_HAS_NUM_OF', 'SYS_HITL')}
inventory_file = REPO/'docs/investigations/20261116_stage1/evidence/ULOG_INVENTORY.json'
classification_file = REPO/'docs/investigations/20261116_stage1/evidence/ULOG_EXTRA_CLASSIFICATION.json'
inventory = json.loads(inventory_file.read_text())
classification = json.loads(classification_file.read_text())
result = {'source': str(SOURCE), 'source_sha256': full['sha256'], 'size_bytes': full['size_bytes'],
          'firmware_and_hardware': full['firmware_and_hardware'],
          'simulator_identity': {'ver_hw': u.msg_info_dict.get('ver_hw'), 'sys_os_name': u.msg_info_dict.get('sys_os_name'),
                                 'SYS_AUTOSTART': all_params.get('SYS_AUTOSTART'),
                                 'is_sitl_from_ulog_metadata': u.msg_info_dict.get('ver_hw') == 'PX4_SITL',
                                 'filename_not_sufficient_for_hardware_classification': True},
          'full_log_duration_s': full['ulog_duration_s'], 'nav_segments': full['nav_segments'],
          'position_interval': segment, 'interval_details': details, 'fusion_parameters': fusion_params,
          'changed_parameters': helper.basic(u.changed_parameters),
          'events_near_interval': [e for e in full['decoded_events'] if e['timestamp_us'] >= lo - 10000000],
          'messages_near_interval': [m for m in full['logged_messages'] if m['timestamp_us'] >= lo - 10000000],
          'distance_sensor_topic_instances': [d.multi_id for d in u.data_list if d.name == 'distance_sensor'],
          'visual_odometry_topic_instances': [d.multi_id for d in u.data_list if d.name == 'vehicle_visual_odometry'],
          'inventory_boundary': {
              'provided_inventory_source': str(inventory_file),
              'inventory_source_sha256': hashlib.sha256(inventory_file.read_bytes()).hexdigest(),
              'provided_classification_source': str(classification_file),
              'classification_source_sha256': hashlib.sha256(classification_file.read_bytes()).hexdigest(),
              'provided_counts': {k:inventory[k] for k in ('all_paths', 'excluded_trash_or_px4_sample', 'selected_paths', 'unique')},
              'selected_inventory_paths_under': '/home/jikl62',
              'path_count_computed_from_file_entries': sum(len(f['paths']) for f in inventory['files']),
              'unique_sha256_count_computed_from_file_entries': len({f['sha256'] for f in inventory['files']}),
              'classified_unique_records': len(classification),
              'records_with_position': [r for r in classification if r['position_intervals']],
              'limitation': 'Provided inventory reports 361 discovered paths, excludes trash/PX4 sample paths, and retains 61 paths / 30 hashes. It lists selected paths but not excluded path names, exact discovery command, unreadable paths, external mounts, or contents of compressed archives. Do not claim a complete machine-wide search or absence of all possible other Position logs.',
              'original_audit_boundary': 'The original 12-file audit intentionally covers only direct *.ulg files in /home/jikl62 and /home/jikl62/桌面; the archived file is outside that scope.'},
          'conclusion': 'Five-second historical simulator failsafe/landing Position interval; small estimator z range does not establish current physical MTF/FC01 Position stable-height baseline.'}
(OUT/'position_interval.json').write_text(json.dumps(finite_json(result),indent=2,ensure_ascii=False,allow_nan=False)+'\n')
(OUT/'initial_parameters.json').write_text(json.dumps(all_params,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(OUT/'position_interval.json')
