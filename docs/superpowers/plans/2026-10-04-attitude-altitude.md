# FC01 attitude-altitude Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Execution is inline as authorized by the supplied handoff.

**Goal:** Ship a complete 0.4.0 source release, defaulting to disarmed zero-thrust attitude preparation.

**Architecture:** Preserve the existing ground ACK core and Runtime ACK handler byte-for-byte. The new pure C++ core consumes the same transaction results through Runtime; the ROS adapter creates publishers by profile and emits setpoint and heartbeat together only for healthy output.

**Tech Stack:** C++17, ROS2 Humble/rclcpp, pinned px4_msgs commit 392e831c1f659429ca83902e66820d7094591410, Python3 operator/deployment scripts.

**Spec:** ../../contracts/03_IMPLEMENTATION_CONTRACT.md and ../../contracts/04_ACCEPTANCE_TESTS.md.

## Global Constraints

- Version: 0.4.0-attitude-altitude; no overwrite of baseline or automatic current switch.
- Stage one: flight_actions_enabled=false, arm_enabled=false, takeoff_enabled=false; zero thrust and never Arm.
- Stage two: explicit reviewed parameters and all configuration/state/interactive-console gates.
- Attitude and trajectory ownership are mutually exclusive; ground owns no setpoint publisher.
- No PX4 main interfaces, board parameters, ESC/PID work, systemd installation, or invented flight gains.
- Quat/Z/VZ reset while active, stale/invalid/NaN telemetry, timestamp regression, failsafe, conflicts and loop stalls latch safety and cease heartbeat.
- Preserve socket security, Agent v2.4.3 build script, ground MODE semantics, legacy navigation behavior and ACK matching.

## Review Focus

- Late Arm ACK or armed feedback after timeout: no automatic recovery/rearm or zero-thrust healthy heartbeat.
- Disarm shares command 400 with Arm: serialize and preserve the last ACK watermark across transactions.
- Repeated telemetry with a new arrival time: receipt does not refresh a repeated source timestamp.
- Console ready command may return before mode confirmation: single-command client must wait for OFFBOARD_READY and release preparation on disconnect.
- Shutdown, console loss and external mode changes: stage one releases control; active flight descends only while telemetry remains healthy, else releases to PX4.

### Task 1: Attitude reference, vertical controller and ownership

**Files:** create include/jikl_fc01_system/{attitude_reference,vertical_controller,setpoint_ownership}.hpp and tests/test_attitude_vertical.cpp.

**Interfaces:** AttitudeReference::observe(q,reset,armed) and level(); VerticalController::step(sample,target,dt); ownership(profile,counts).

- [x] Write independent numeric tests for level/fixed-yaw quaternion, normalization, invalid values, disarmed/armed resets, NED signs, hover feed-forward, correction direction, clamp, slew, anti-windup and stale/reset handling.
- [x] Run test and observe failure before adding core implementation.
- [x] Implement finite validated quaternion math, cascaded h/v controller, conditional integration and clamp/slew limits with explicit sentinel flight config.
- [x] Run test.sh and compare PASS output; commit.

### Task 2: State machine and existing transaction integration

**Files:** create attitude_altitude_core.hpp; modify runtime.hpp; create tests/test_attitude_runtime.cpp.

**Interfaces:** AttitudeCore::prepare/request_takeoff/descend/step consumes AttitudeFeedback + transaction terminal state; Runtime::step returns attitude output and starts transactions via the original track() implementation.

- [x] Write tests for zero-thrust preparation, continuous prestream >=1.2 s, accepted ACK plus newer status, all flight gates, interactive lease, all telemetry/clock/reset faults, full disabled-by-default flight lifecycle and serialized Arm/Disarm.
- [x] Observe RED for missing profile and behavior.
- [x] Implement routing and state gates; preserve Runtime::ack(), track() and ground_link.hpp unchanged.
- [x] Run all C++ suites, including original ACK lifecycle tests; commit.

### Task 3: ROS adapter, console and deployment

**Files:** modify fc01_system_node.cpp, tools/fcctl.py, tools/deploy.py, scripts/{run,install,export_portable,test}.sh, package.xml, VERSION; add config/attitude_altitude.yaml and tests/test_tools.py/test_socket.cpp.

**Interfaces:** new operator ATTITUDE_READY, live TTY INTERACTIVE marker plus PING lease, profile-specific publishers and configurable logs/topics; shared Runtime ACK callback retained.

- [x] Write behavior tests for single-command flight rejection, interactive readiness completion, socket permissions, release naming/no current switch and source-only portable export.
- [x] Observe failures against baseline scripts.
- [x] Add ROS VehicleAttitude/VehicleAttitudeSetpoint adapter with v1.16 static assertions, monotonic receipt stamps, paired health-gated output and profile-dependent ownership checks.
- [x] Add YAML sentinel parameters, default locks, live-console marker, release version and installation without any current switch.
- [x] Run all tests, Python/shell syntax and strict C++ core compilation; report real ROS build UNKNOWN if unavailable; commit.

### Task 4: Review and deliver

**Files:** README_zh.md, docs/{CONFLICTS,ACCEPTANCE_RESULTS,PORTABILITY_AUDIT,F_DELIVERY_REPORT}.md; source tar.gz/zip and baseline diff.

- [x] Record exact verified facts/conflict rulings, evidence limits and all required smoke/build/rollback steps.
- [x] Review complete diff and all contract requirements; address significant findings with RED/GREEN tests.
- [x] Run final test.sh and record output; verify ACK/ground/socket invariants and archive byte consistency.
- [x] Package source-only release plus unified baseline diff, verify patch replay against the baseline, and save deliverables.
