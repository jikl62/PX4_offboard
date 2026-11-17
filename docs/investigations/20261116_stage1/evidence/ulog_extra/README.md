# Archived Position interval supplement, 2026-11-16

**The newly found Position interval is simulator evidence. It does not establish a current physical FC01/MTF Position height baseline.** The ULog reports `ver_hw=PX4_SITL`, `sys_os_name=Linux`, and `SYS_AUTOSTART=4001`, despite the filename lacking “SITL”. Its Position state occurs for 5.004 seconds during failsafe and includes a landing detection before disarm.

Source: `/home/jikl62/DroneProjects/05_飞行日志与测试证据/原下载目录/FC01_M1_S10_failure.ulg`.

- SHA-256: `21397a1b4e313a63a6b8401f5632218aa785a0c4212a88dbe8a30612a0b4b9d1`.
- Size: 44,282,324 bytes. Fully parsed ULog duration: 157.920 seconds, from boot 3.712 to 161.632 seconds.
- Firmware hash: `8714f2442ae63e47c9f23fd924a7106f09a9f634`, different from the `07983bfb...` hash in the twelve direct-path recordings. Reported software release `17826304` is 0x01100200 (v1.16.2 encoding); firmware version similarity does not make a simulator recording a physical hardware baseline.

## Position interval observations

The recorded sequence is AUTO_LOITER (`4`) → OFFBOARD (`14`) → POSCTL (`2`) → OFFBOARD (`14`). The Position interval is boot `[155.632000, 160.636000)` seconds. All 10 `vehicle_status` samples within it have `nav_state=2`, `arming_state=2` (armed), and `failsafe=1`. At 160.636 seconds, status changes to disarmed and Offboard; that final Offboard interval is not armed flight.

| Observation during Position | Recorded result |
| --- | --- |
| Local-position samples | 625, boot 155.636 to 160.628 seconds |
| Local z, NED m | -0.053611532 to -0.037248656; span 0.016362876 |
| vz, NED m/s | -0.004835952 to +0.001920886 |
| z_deriv, NED m/s | -0.003032013 to +0.014129810 |
| z_valid / v_z_valid | 1 for all 625 samples |
| z / vz reset counters | Constant 3 / 2 |
| Attitude reset counter | Constant 1 in 1,251 attitude samples |
| dist_bottom, m | 0.099999994 to 0.139022052 |
| dist_bottom_valid | 0 for all 625 samples |
| Distance-sensor topic | No instance recorded |

The last landed state before Position is false at boot 154.724 seconds. Within Position, ground contact becomes true at 157.940 seconds, maybe_landed at 158.292 seconds, and landed at 158.628 seconds. The console and decoded event report “Landing detected” at 158.632 seconds. Disarm by landing occurs at 160.636 seconds. The near-zero z range therefore includes approximately two seconds explicitly marked landed. It is not evidence of a five-second airborne height hold.

The event at 155.632 seconds says “Failsafe activated: switching to Position mode”. A console warning at 155.652 seconds says “Matching flight task was not able to run, Nav state: 2, Task: 1”. Thus the observed `nav_state=2` does not by itself demonstrate that a matching Position flight task successfully controlled the vehicle. The final logged Offboard heartbeat is boot 154.597811 seconds, preceding Position by about 1.034 seconds; this ordering alone does not identify the underlying failure cause.

## Height fusion and sensor configuration

All initial parameters are preserved in `initial_parameters.json`. No parameter-change messages are recorded. Relevant settings are `EKF2_HGT_REF=1`, `EKF2_GPS_CTRL=7`, `EKF2_BARO_CTRL=1`, `EKF2_EV_CTRL=0`, `EKF2_RNG_CTRL=1`, and `EKF2_OF_CTRL=1`. The configuration also reports `SYS_HAS_GPS=1`, `SYS_HAS_BARO=1`, `SYS_HAS_NUM_DIST=0`, and `SYS_HAS_NUM_OF=0`.

Parameter enablement alone does not establish sensor presence or fusion. In the interval, estimator status flags have `cs_baro_hgt=1`, `cs_gps_hgt=1`, and `cs_gnss_pos=1`, with `cs_rng_hgt=0`, `cs_opt_flow=0`, and external-vision height/position/yaw/velocity flags zero. Barometer height has 83 aid-source records and GNSS height has 152, all `fused=1` and `innovation_rejected=0`. No range-height or external-vision aid-source topic and no visual-odometry topic are recorded. These data support a simulated barometer/GNSS fusion context; they provide no validation of physical MTF range, optical flow, or external-vision performance.

## Inventory boundary

The original audit in `/tmp/fc01-stage1-ulog-audit/` intentionally covers only the 12 `*.ulg` files directly inside `/home/jikl62` and `/home/jikl62/桌面`. Its “no Position observed” result is correct for that explicit boundary. This archived file is outside it.

The root agent's expanded inventory is recorded in repo `docs/investigations/20261116_stage1/evidence/ULOG_INVENTORY.json` and `ULOG_EXTRA_CLASSIFICATION.json`. It reports 361 discovered paths, 300 excluded trash/PX4 sample paths, 61 retained paths, and 30 unique SHA-256 values. All 61 retained path entries are under `/home/jikl62`; copies in quarantine and project archives are deduplicated by content hash. The 30 classification entries contain one Position-bearing hash: the simulator file analyzed here. `is_sitl_label=false` for this filename means only the label lacks SITL; the ULog metadata establishes simulator identity.

The inventory JSON lists retained paths but does not include excluded path names, the exact discovery command, read-error records, external-mount coverage, or the contents of compressed archives. Its results describe that retained inventory; they must not be expanded into a claim that no other Position recording exists anywhere. This supplement directly parses the identified archived recording and does not repeat a machine-wide search. Inventory file content hashes and counts are preserved in `position_interval.json`.

## Artifacts and reproduction

`position_interval.json` contains the explicit interval boundary, local statistics, reset/validity/state changes, prior samples for sparse state topics, fusion settings, all relevant topic field summaries, and nearby events/messages. `desktop_FC01_M1_S10_failure.json` and its local-position CSV preserve the existing scan helper's full-log evidence. `inspect_extra.py` uses the included copy of `inspect_ulogs.py`; neither parser opens hardware or retrieves event metadata from the network. Nonfinite values in sparse topic snapshots are written as JSON null; numeric summaries retain nonfinite sample counts.

```bash
python3 /tmp/fc01-stage1-ulog-extra/inspect_extra.py
```

The small, finite estimator z range and constant counters are confirmed observations for this historical simulator failsafe/landing interval. They do not establish a current physical airborne stable-height baseline, sensor accuracy, MTF fusion, current companion/PX4 pairing, or the cause of any current hardware failure.
