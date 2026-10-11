# Verified field baseline as of 2026-10-05

## 0.4.1 build and identity
On the user's Ubuntu 22.04 + ROS 2 Humble PC, `0.4.1-safety-hardening` was actually built against pinned `px4_msgs` commit:

`392e831c1f659429ca83902e66820d7094591410`

Build result:
- px4_msgs: PASS
- jikl_fc01_system: PASS
- `Summary: 2 packages finished`
- built release: `/home/jikl62/fc01_system/releases/0.4.1-safety-hardening-20261005T054041Z-NjH6I4`

All four flight gates remained false:
- flight_actions_enabled=false
- arm_enabled=false
- takeoff_enabled=false
- flight_parameters_confirmed=false

## 0.4.1 real FC01 stage-one smoke: PASS
With real FC01, ROS2/XRCE-DDS and battery connected, but no Arm and zero thrust:
- initial nav_state=15 (Stabilized), armed=false, landed=true, at_rest=true
- `attitude-ready` entered SELF_CHECK -> PRESTREAM
- heartbeat_enabled=true and attitude_output_enabled=true in PRESTREAM
- local yaw reference captured
- stage-one thrust_body remained exactly `[0,0,0]`
- VehicleCommand 176 accepted: `offboard_ack=0`
- actual VehicleStatus reached nav_state=14 (Offboard)
- terminal phase reached OFFBOARD_READY
- armed=false throughout
- grep found `NO_ARM_COMMAND_FOUND`

Therefore real Humble build/link + real DDS/FC01 stage-one path is no longer UNKNOWN for 0.4.1.

## Disarmed Offboard-loss observation
On the earlier 0.4.0 smoke, after client release:
- companion returned phase=IDLE and stopped attitude output/heartbeat
- PX4 `offboard_control_mode` publication rate became 0 Hz
- `failsafe_flags.offboard_control_signal_lost=true`
- vehicle remained Disarmed, navigation/user intention still Offboard, failsafe=false
- operator manually changed to Stabilized; commander then reported Disarmed / Stabilized / no failsafe

Do not assume a disarmed Offboard-loss automatically changes nav_state.

## Current sensor/control facts
Real FC01 telemetry observed:
- attitude fresh/valid
- Z valid, VZ valid
- XY invalid, VXY invalid
- heading_good false
- landed/at_rest available
- battery connected around 8.3–8.5 V during later tests

## Current PX4 parameter evidence
Observed on FC01:
- MPC_THR_HOVER = 0.5000
- MPC_USE_HTE = 1
- MPC_THR_MIN = 0.1200
- MPC_THR_MAX = 1.0000
- MPC_Z_VEL_MAX_UP = 3.0000
- MPC_Z_VEL_MAX_DN = 1.5000
- MPC_TILTMAX_AIR = 45 deg
- MPC_TILTMAX_LND = 12 deg
- COM_RC_OVERRIDE = 1
- COM_RC_IN_MODE = 4
- COM_RC_STICK_OV = 30%
- COM_RCL_EXCEPT = 0
- COM_OF_LOSS_T = 1.0 s
- COM_OBL_RC_ACT = 0
- COM_DISARM_LAND = 2.0 s
- COM_SPOOLUP_TIME = 1.0 s

`mc_hover_thrust_estimator status` runs, but `hover_thrust_estimate` had never published at the observation time. Do not treat MPC_THR_HOVER=0.5 as a measured real hover value.

## RC evidence
At the time of this handoff, the physical RC receiver was NOT connected. Therefore:
- `input_rc`: never published
- `manual_control_switches`: never published
- RC link, Channel 5 Kill, Channel 6 Arm and stick takeover are UNKNOWN, not failed

User requirement is explicit: RC takeover is mandatory as one selectable operational route. Autonomous/no-RC must also remain a separately selectable route.
