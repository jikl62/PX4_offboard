# R6 control recovery implementation plan

> **For agentic workers:** Use superpowers:executing-plans; implement inline.

**Goal:** remove immediate descent-timeout stream loss and disclose bounded recovery/handoff failure honestly.
**Architecture:** extend existing Core phases and Runtime command 21 transaction; preserve final sink guards and all R4 ACK protection.
**Tech Stack:** C++17, Python3, PX4 1.16 pinned messages, ROS2 Humble adapter.
**Spec:** ../specs/2026-10-09-r6-control-recovery-design.md

## Global constraints

Independent R6 copy, R5 inputs immutable. No true FC01 operations. Four default gates false.
No guessed physical thrust/gains. No modified C counterexamples. No unbounded recovery or airborne routine Disarm.

## Review focus

- Repeated LAND/shutdown cannot reset recovery deadlines.
- Delayed 21 ACK or stale AUTO_LAND cannot claim a safe handoff.
- Target commits but heartbeat throws must block every command and log partial success.
- Invalid Z/VZ, RC Kill, Agent loss or hard termination cannot be labeled a controlled descent.
- Bounce after submitted Disarm must retain Command400 ambiguity with no retry.

### Task 1: bounded recovery and transaction evidence

Files: attitude_altitude_core.hpp, runtime.hpp, publication_guard.hpp, publication_boundary.hpp;
tests/test_m1_r6_recovery.cpp, retained original C-S10 gates.
Interfaces: RecoveryDescent/Px4LandPending phases; Px4Land command; Core step accepts independent 21 Transaction.

- [ ] Write new positive timeout/shutdown regressions and run against R5; expect missing output at timeout.
- [ ] Add single absolute recovery budget, optional guarded 21, and honest responsibility states.
- [ ] Run positive/negative, deadline, stale mode/ACK, bounce and all original R4 tests.

### Task 2: real publication observation and adapter

Files: publication_audit.hpp, fc01_system_node.cpp, attitude_ros_parameters.hpp;
tests/test_r6_publication_audit.cpp, tests/adapter_compile_check.py (schema fixture only).
Interface: per-sink local commit audit, phase/ownership JSONL event; no PX4 receipt claim.

- [ ] Write partial-sink/gap regression before audit implementation.
- [ ] Track target before heartbeat, handle exceptions per sink, block subsequent command commits.
- [ ] Log monotonic time, current PX4 evidence, thrust, sinks, commands and phase on every tick.
- [ ] Run fixture compile and offline suite; native Humble/DDS remain UNKNOWN if unavailable.

### Task 3: default configuration, offline checks and delivery

Files: config/R6_OFFLINE_DEV.yaml, config/R6_REAL_FC01_PENDING_REVIEW.yaml,
tools/r6_preflight_audit.py, scripts/r6_no_props_check.sh, docs/R6_* and C_REVIEW_TASK.md.

- [ ] Test disabled/default/unreviewed configurations and read-only event validation.
- [ ] Add check-only script without ROS command publishers or Arm/parameter writes.
- [ ] Run whole suite, unchanged C gates and RED/GREEN, compare input SHA and px4_msgs.
- [ ] Fresh code review, address material findings, package source/patch/evidence and SHA256.
