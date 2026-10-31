# Orange Pi RC navigation takeoff

## Intent and authorization

The user has selected independent RC arming (CH5), navigation on CH6 mode slot 6 (PX4 Offboard), and CH7 kill. They now explicitly request the control state machine and background controller implemented, with questions only for genuinely missing decisions. The previously discussed sequence was corrected against PX4 v1.16 Commander: RC arm must occur in Stabilized BEFORE selecting Offboard. Existing supplied sources, RC definitions and numeric defaults suffice; no additional terminal paste is required before implementation.

Provide an offline, one-command upgrade that runs the real C++ controller automatically at boot, prepares Offboard traffic while disarmed in Stabilized, observes an authorized manual arm, and takes off only after a new CH6 navigation selection and actual PX4 Offboard confirmation. Visible desktop status addresses the user's expectation of seeing the program. The first target is relative height 0.5 m, attitude leveling with fixed captured yaw; this is not horizontal position hold. Vision remains a later feature.

## Approach

Extend the supplied native attitude Core and final publication boundary behind a default-false `rc_navigation_enabled` option. Reuse its vertical controller, feedback guard, estimator/reset/tilt checks, landing and ground-disarm verification; preserve the legacy interactive route when the option is false. A Python supervisor handles Agent/process ownership, bounded logs, status and installation, never flight setpoints. A fresh C++ startup path observes RC inputs directly; it does not fabricate an interactive console lease or send ARM commands.

Replacing the native safety controller with a Python setpoint script would duplicate existing protections. Driving the old console with synthetic INTERACTIVE/PING/TAKEOFF would retain program arming and the wrong mode order. These alternatives conflict with the chosen independent-arm behavior.

## Defaults and boundaries

- PX4 1.16 pinned message definitions; ROS 2 Humble on Ubuntu 22.04 ARM64. Existing native source and base installation remain unchanged. All edits live in a new packaged native overlay.
- Pi `/dev/ttyS2`, 115200 baud, domain 0, localhost-only 0. Reuse a matching same-user Agent, preserve unrelated owners and existing controllers.
- Default relative target 0.5 m; control 20 Hz; existing task limit 30 seconds (includes climb/hold), followed by existing supervised descent. Height/tilt limits 5 m/10 degrees, climb/descent .3/.2 m/s, candidate hover .475 and reserve .040 remain unchanged. This is not new physical calibration.
- CH6 mode slot 6 selects Offboard; other slots are non-navigation. Actual arming mode must be Stabilized (nav_state 15). CH5 arm OFF=3, ON=1; CH7 kill OFF=3, ON=1, per supplied pinned ManualControlSwitches.
- Minimum throttle (`<= -.9`), centered roll/pitch/yaw (`abs <= .15`), no moving sticks, fresh selected RC source and switch telemetry are required through ground admission and navigation entry. Nonfinite axes cannot authorize preparation.
- Keep all original freshness windows and vertical-feedback tolerance/dwell. The prior VERTICAL_VELOCITY_INCONSISTENT and occasional freshness loss are not declared fixed or bypassed. Report exact inhibiting conditions; no GPS/XY requirement is added to attitude-height control.
- `flight_enabled=false`, `flight_actions_enabled=true`, `arm_enabled=false`, `takeoff_enabled=true`, flight_parameters_confirmed and rc_takeover_confirmed true in this explicitly requested RC-flight profile; autonomous/bench flags false. C++ validation and final ARM sink independently reject every automatic ARM when RC navigation is enabled. Ground-verified automatic DISARM after landing remains supported.

## Native state machine

Add explicit RC preparation phases after the existing enum range, preserving legacy ordering-dependent comparisons. Before joining flight, use a dedicated RC startup step; after joining reuse ArmedGround -> ThrustRamp -> Ascend -> HeightHold -> Descent -> ground proof -> Disarm -> SafeDisarmed.

1. **Idle / wait safe reset:** no output if boot already armed, already Offboard, arm switch ON, kill asserted/unknown, sources invalid or sticks unsafe. A held navigation selection never starts a mission. Require actual disarmed, landed, at-rest, Stabilized, arm OFF and non-navigation selection, healthy sources and existing 2-second ground dwell.
2. **RC_PRESTREAM:** capture disarmed Z/yaw/reset counters. Send exact-zero thrust, level attitude and attitude-mode heartbeat, under a new final zero-only startup policy. Stabilized remains pilot-owned. Rebase only on current validated disarmed/arm-OFF ground observations. Warmup readiness requires actual successful target+heartbeat local commits for >=1.2 s, contiguous gaps <= the existing .3 s stall bound, sufficient commits for the 20 Hz schedule, and PX4 reporting offboard signal available. Local publish alone is not receipt proof.
3. **RC_READY_WAIT_ARM:** retain prestream while awaiting a new OFF->ON arm-switch observation. Freeze ground reference as soon as arm intent or armed state is observed; never recapture after that. Accept manual arming only with a new fresh PX4 armed status after the last disarmed baseline, advancing PX4 armed_time, latest_arming_reason=RC_SWITCH(2), and the matching fresh RC arm transition. Handle either arrival order of status and switches within the existing 2.5-second telemetry window. Other arming reasons, pre-ready arm, airborne acquisition or stale evidence inhibit the mission. No command 400 Arm is generated.
4. **RC_ARMED_WAIT_NAVIGATION:** retain exact-zero stream while still on verified ground; accept a new non-navigation->slot6 transition after the arm intent, then actual fresh armed Offboard status. Require current arm ON, kill OFF, neutral sticks/minimum throttle, valid reference/reset counters and original ground Z/Vz bounds. Allow up to 8 seconds from manual-arm observation for the navigation selection; timeout latches the attempt for operator reset, with no forced arm/disarm or mode request.
5. **Join flight:** only with all independent RC/ground/prestream/mode evidence, set flight_started/arm_confirmed and lock RC authority. Enter existing ArmedGround with frozen Z/yaw, target .5 m, zero initial thrust, and the accepted switch-generation baseline. Emit neither Arm nor an Offboard-entry 176; the pilot owns both transitions. Refresh operator lease from actual RC facts, not terminal pings. Final sink resamples RC/mode/safety data; startup phases can never emit nonzero thrust.

The before-flight preparation code may automatically retry health admission while safely disarmed; it never automatically arms. A fault after arm intent or flight is latched. One flight attempt per native process. A completed or failed process may exit/restart only after fresh disarmed/landed/rest evidence plus CH5 OFF and CH6 non-navigation; each new process must repeat safe ground/prestream/manual-arm/new-navigation admission. Holding switches cannot replay a mission.

## Handover, stop and faults

- CH6 leaving navigation cancels the mission. Actual PX4 departure confirms handover; if still armed Offboard during the transition, retain the existing guarded controlled-descent path until departure/recovery is observed. Never request Offboard back. Existing kill, failsafe, stale-source and estimator fault policies remain.
- CH5 OFF, other RC switch changes or moving sticks after navigation are intervention, not fresh takeoff authorization. Do not pretend a switch event alone proves PX4 changed mode.
- UI closure is read-only and does not affect flight. Supervisor loss/socket loss/native SIGTERM requests existing supervised descent when flight is active, preserving output custody and Agent until native recovery completes. Do not force-kill. Before flight, cancellation withdraws preparation and requires a fresh safe reset; it never commands an unexpected routine disarm.
- A physical/local data fault may prevent controlled descent; retain the existing explicit PX4/operator responsibility instead of promising a hover using bad vertical feedback. Preserve exact reason/evidence in status and bounded logs.

## Source, build and service

Package a new native overlay (headers, node, CMake, config-check executable and tests), without editing the installed candidate. On the Pi, source the known Humble/base px4_msgs environment and compile only this small native package with CMake, one build job. Do not download dependencies or rebuild px4_msgs. Detect missing compiler/CMake/headers and preserve a single installation report rather than asking for many manual probes. A separate config-check executable consumes the actual generated YAML through rclcpp without setpoint/command publishers.

Before changing the old managed standby service, obtain fresh unarmed/landed evidence through a passive status/land subscriber (may reuse/start the known matching Agent, never kill unrelated Agents). Compile/validate first, then stop+disable only the known FC01 standby service and enable the new `fc01-navigation.service`. Refuse unmanaged same-name services and differing payloads. Preserve old files and offer rollback to the old closed-gate service. Installation/enable never authorizes arming; startup admission still requires safe switch positions and new edges.

Use the shared operator lock and process scan. Retain deliberate KillMode=process / no SIGKILL, with supervisor-owned graceful native shutdown and safe Agent reuse. Retained old native blocks duplicate startup. Native logs 4 MiB plus two backups; supervisor logs 1 MiB plus two backups; CSV disabled by YAML empty string, ROS external file logging disabled. Snapshots refresh every 5 seconds including retry waits. Heartbeat control does not depend on logging/terminal I/O.

## Visible operation and packaging

Install a desktop shortcut and an XDG desktop-login autostart entry for a read-only terminal status view. Controller boot is independent of desktop login. Display RC phase, inhibit reason, actual armed/mode/height, target, heartbeat commit evidence, and last update. No green READY for generic CONNECTED while Safety is latched. Window closure does not stop the controller. Provide `status`, `monitor`, `logs`, `report`, `start`, `stop`/`land`, `disable`, and rollback management; stop/land uses native graceful shutdown, never kill or direct disarm.

Deliver one offline self-extracting `FC01_OrangePi_Navigation_20261028.sh` with integrity checks, source/tests/docs, one install command, automatic source build and unit setup, bounded run records and a diagnostic ZIP on failure. Clearly distinguish host verification, on-device compilation/validation, and physical hover acceptance. No hardware-flight or vertical-sensor fix claim without new device evidence.
