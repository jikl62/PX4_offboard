# Formal Task — Agent G: FC01 0.4.1 Safety Hardening

## Role
You are the implementation engineer for **offline safety hardening** of the FC01/PX4 ROS2 companion controller.

Base candidate: `0.4.0-attitude-altitude`.
Output candidate: `0.4.1-safety-hardening`.

This is NOT a flight experiment. You have no authority to connect/control a real vehicle, change the user's active release, or enable flight actions.

## Mandatory inputs
Read fully before editing:
- `00_README_FIRST.md`
- `02_VERIFIED_BASELINE_AND_NEW_EVIDENCE.md`
- `03_IMPLEMENTATION_CONTRACT.md`
- `04_ACCEPTANCE_TESTS.md`
- `05_RELEASE_OUTPUT_CONTRACT.md`
- actual `candidate/` source
- actual `baseline/` source
- D independent review
- F delivery report
- A/B/C/E reports as supporting evidence

## Primary mission G1
Close D finding **I-01** without changing the overall Attitude-Offboard architecture.

### A. Four-gate defense at final dangerous publisher boundaries
The four gates are:
- `flight_actions_enabled`
- `arm_enabled`
- `takeoff_enabled`
- `flight_parameters_confirmed`

Config/state-machine checks are not enough. Add final-boundary defense in depth.

#### Arm publication boundary
Before publishing any `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=1`, independently require at least:
- profile == `attitude_altitude`
- all four gates allow the operation
- correct explicit arm phase/state
- shutdown latch false
- live interactive operator lease/session valid
- currently disarmed
- required telemetry/health constraints still satisfied

Any failure => fail closed, publish no Arm command, emit an explicit diagnostic reason.

#### Non-zero thrust publication boundary
Before any non-zero `VehicleAttitudeSetpoint.thrust_body` reaches the ROS publisher, independently require the complete flight gate + allowed flight phase.

For stage one (`IDLE/SELF_CHECK/PRESTREAM/REQUEST_OFFBOARD/OFFBOARD_READY`), the final boundary must guarantee exactly:
`thrust_body = [0,0,0]`.

If an upstream bug supplies finite non-zero thrust in stage one, do NOT silently clamp it to zero and continue as healthy. Reject/fail closed and surface a clear reason.

### B. Keep stage-one behavior unchanged
The following must remain possible with all flight gates false:
`IDLE -> SELF_CHECK -> PRESTREAM -> REQUEST_OFFBOARD -> OFFBOARD_READY`
with:
- no Arm
- no Takeoff
- exact zero thrust
- attitude-only ownership
- local yaw capture + level roll/pitch
- health-gated heartbeat/setpoint

Do not accidentally make `attitude-ready` impossible just because flight gates are false.

### C. Status observability
Expose all four gates in status/JSON diagnostics, including `flight_parameters_confirmed`.
Also expose/retain, without breaking existing consumers if practical:
- profile
- phase
- armed
- nav_state
- heartbeat/output enabled
- thrust_body
- four flight gates

### D. Stale Unix socket hardening
A real activation attempt showed:
- no `fc01_system_node` / `MicroXRCEAgent` process
- user service inactive
- path `~/.cache/fc01/control.sock` existed and was a Unix socket
- no listener in `ss`
- `update.sh` failed safely on `ECONNREFUSED`
- after a human verified it was stale and removed the socket, update succeeded

Design safer handling.

Hard requirements:
- never delete merely because connect returns `ECONNREFUSED`
- verify filesystem object is a Unix socket; regular file/symlink/unexpected type => fail closed
- avoid race with a process starting/stopping
- if liveness cannot be proven safely/portably, prefer precise diagnosis + manual remediation instead of auto-delete
- no blind `/dev`/process heuristics that could kill/remove unrelated runtime state

You may decide the safest implementation is **diagnostic-only stale detection** rather than automatic deletion. Explain the rationale.

### E. Preserve verified invariants
Must not weaken or redesign:
- 0.3.1 ACK correlation semantics
- target `1/191`, pending/freshness/watermark/timeout behavior
- `from_external=false` acceptance semantics
- profile setpoint ownership separation
- install builds candidate but never auto-switches `current`
- release immutability / rollback model
- PX4 v1.16 pinned message assumptions

## Absolute prohibitions
Do NOT:
- modify 0.4.0 in place
- call the user's real `update.sh`
- change the user's `current`
- start real Agent/controller/FC01 chain
- send `attitude-ready` to real hardware
- Arm
- Takeoff
- generate real non-zero setpoints
- write PX4 parameters
- invent hover thrust/gains/limits
- investigate ESC/ULog/motor 2/4 tuning
- install/configure Orange Pi
- use PX4 main to override v1.16 definitions

## Output version
- release: `0.4.1-safety-hardening`
- package.xml version: `0.4.1`

At the end, produce complete source archives + patch + report as defined in `05_RELEASE_OUTPUT_CONTRACT.md`.
