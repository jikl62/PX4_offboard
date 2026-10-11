# K pinned px4_msgs schema review

Pin reviewed: `392e831c1f659429ca83902e66820d7094591410` (PX4/px4_msgs), not PX4 main.

## ManualControlSetpoint.msg

The exact pinned message defines:

- `MESSAGE_VERSION = 0`
- `uint64 timestamp`
- `uint64 timestamp_sample`
- `bool valid`
- `SOURCE_UNKNOWN = 0`
- `SOURCE_RC = 1`
- `SOURCE_MAVLINK_0..5 = 2..7`
- `uint8 data_source`
- roll/pitch/yaw/throttle/flaps/aux1..aux6
- `bool sticks_moving`
- `uint16 buttons`

Source checked directly at the immutable commit path:
`PX4/px4_msgs@392e831c1f659429ca83902e66820d7094591410/msg/ManualControlSetpoint.msg`.

This is sufficient for RC source readiness without `InputRc`: the adapter can require an advancing raw `timestamp_sample`, a fresh envelope `timestamp`, `valid=true`, and `data_source==SOURCE_RC`.

## ManualControlSwitches.msg

The same pinned source defines `timestamp`, `timestamp_sample`, switch position constants (`NONE=0`, `ON=1`, `MIDDLE=2`, `OFF=3`), `mode_slot`, `arm_switch`, `kill_switch`, the other switch fields, and `switch_changes`.

Source checked directly at:
`PX4/px4_msgs@392e831c1f659429ca83902e66820d7094591410/msg/ManualControlSwitches.msg`.

## `sticks_moving` decision

PX4 v1.16 `ManualControl.cpp` computes `sticks_moving` from changes in roll/pitch/yaw/throttle against the `COM_RC_STICK_OV` threshold and publishes it in the selected manual-control setpoint. That makes it useful diagnostic evidence of fresh selected-source stick activity, but it is not by itself proof that PX4 actually performed an Offboard takeover; the actual Offboard override path also depends on authority/failsafe policy and parameter configuration.

Therefore 0.4.3 exposes `rc_sticks_moving` only as diagnostics. It does **not** use that bit alone to satisfy RC_TAKEOVER, clear Kill, request Offboard, Arm, or authorize nonzero thrust. Existing stronger takeover paths (nav-state departure, `failsafe_and_user_took_over`, switch generation, Kill, RC readiness loss, PX4 failsafe, Offboard-signal loss) remain unchanged.
