# Authority modes implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task by task. The supplied formal implementation handoff authorizes completion in this session.

**Goal:** Deliver an isolated 0.4.2 candidate supporting RC_TAKEOVER and AUTONOMOUS_NO_RC with fail-closed final publishers.

**Architecture:** A pure authority policy evaluates explicit mode, locked attempt identity and route-specific inputs. The existing attitude Core owns selection/locking/takeover state; Runtime and the actual ROS adapter provide fresh input and final-boundary context. No PX4 modes, parameters or ACK matching are replaced.

**Tech Stack:** C++17, rclcpp/POSIX, Python operator console and offline tests.

**Spec:** `docs/i_contracts/01_I_TASK_SPEC.md`, `03_AUTHORITY_CONTRACT.md`, `04_ACCEPTANCE_TESTS.md`; implementation decisions in `docs/I_DESIGN.md`.

## Global constraints

- Default authority UNSET; four flight gates and both authority confirmation flags false.
- No real FC01/Agent, Arm/Takeoff, PX4 parameter writes or real current changes.
- Preserve numerical flight sentinels and exact pin 392e831c1f659429ca83902e66820d7094591410.
- Preserve stage-one exact zero, final four gates, telemetry/ownership/session/envelope checks and ACK command-400 sequencing.
- Authority mutates only in safe IDLE from a live session; lock by SELF_CHECK.
- Pilot/failsafe takeover stops outputs and cannot automatically reacquire Offboard.
- Pure source archives, clean patch replay with byte/mode equality; real integration missing evidence is UNKNOWN.

## Review focus

- First/missing/duplicate RC or switch sample must never imply a live calibrated RC route.
- Mode departure before terminal Offboard ACK must not cause reacquisition.
- An Arm transaction awaiting acknowledgement must prevent ground reset even if telemetry says disarmed.
- Fault and publication state sampled between a console request and final sink must veto dangerous output.
- Autonomous operation must ignore unavailable RC readiness while still responding to observed kill/failsafe/intervention.

### Task 1: Authority policy and final boundaries

**Files:** Create `include/jikl_fc01_system/authority.hpp` under src/jikl_fc01_system; modify attitude Config/Feedback, publication_guard.hpp; create `tests/test_authority_boundary.cpp`.

**Interfaces:** `authority::Mode`, `State`, `Inputs`, `Reason`, `evaluate(State, Inputs, bool require_lock)` and `name(...)`; BoundaryContext carries State. Config adds `rc_takeover_confirmed` and `autonomous_failsafe_confirmed`; Feedback adds distinct RC/switch facts and intervention indications.

- [x] Write/run an executable boundary regression against the baseline: a complete old boundary without authority must refuse Arm/nonzero (expected RED assertion).
- [x] Test UNSET, wrong/changed/unlocked attempt, both confirmation gates, fresh RC requirement, complete autonomous without RC, and all original four gates and exact stage-one zero.
- [x] Implement the pure policy; re-evaluate at final Arm/nonzero. Preserve original stage-one guard order and paired sinks.
- [x] Run strict C++ boundary tests; adapt only old synthetic flight contexts to explicit locked authority.

### Task 2: Selection, takeover and safe recovery

**Files:** Modify attitude_altitude_core.hpp and runtime.hpp; create `tests/test_authority_runtime.cpp`; adapt old synthetic flight fixtures in test_attitude_runtime.cpp.

**Interfaces:** Core `authority_state()`, `select_authority(...)`, `reset_authority(...)`, `release_to_operator(...)`; Runtime `AUTHORITY_STATUS/RC/AUTONOMOUS/RESET/TAKEOVER` request operations and `authority_json(...)`. Runtime records generated output activity; Snapshot additionally carries actual output/fault facts and RC telemetry facts.

- [x] Write/run default UNSET takeoff admission and live safe selection tests (expected RED refusal/allow mismatch).
- [x] Implement locking by successful prepare, immutable attempt identity, explicit safe ground reset to IDLE/UNSET with outputs off and no pending command.
- [x] Implement one-way takeover for mode departure, explicit event, kill/termination, PX4 failsafe and established Offboard signal loss. RC loss in active RC flight releases; lost session retains existing healthy controlled landing exception but cannot authorize a new attempt.
- [x] Cover every flight phase, pending Arm/Disarm, safe completion, takeover recovery and restored healthy inputs never auto-reacquiring.
- [x] Run full original suite with necessary synthetic authority fixture adaptations only; ACK/track stays byte-identical.

### Task 3: Actual ROS/console wiring and harness

**Files:** Modify fc01_system_node.cpp, attitude_ros_parameters.hpp, config/attitude_altitude.yaml, tools/fcctl.py, VERSION/package.xml; add RC adapter helper/tests, console tests and a real Runtime/final-sink authority harness supplement.

**Interfaces:** `authority rc/autonomous/status/reset/takeover` maps to zero-argument protocol ops. Live selection/reset is rejected before socket creation for one-shot commands. RC adapter derives readiness from InputRc signal stamps, flags, switch freshness and exclusive sources.

- [x] Write/run console parsing and one-shot refusal tests (expected RED feature absent).
- [x] Add observation-only RC/switch subscriptions and termination/offboard-loss mapping. Missing RC must not block UNSET stage-one or autonomous policy; mapped live Kill stops either route.
- [x] Expose mode, lock, attempt, readiness, confirmations and stable refusal/takeover reason in JSON.
- [x] Supplement G2 with deterministic real 0.4.2 Runtime and final publisher sinks, no parallel control model; run original frozen G2 separately and state its baseline scope.
- [x] Run adapter fixture syntax, all C++/Python tests and deterministic scenario rerun; real Humble build remains UNKNOWN if unavailable.

### Task 4: Review and release integrity

**Files:** I_DELIVERY_REPORT, acceptance matrix, invariant manifest, raw test evidence, source archives, patch and SHA256SUMS/PACKAGE_VERIFICATION.

- [x] Have one fresh reviewer inspect actual source/diff/tests against supplied contract; record/fix material findings with regression tests.
- [x] Verify protected ACK blocks, socket/update/install and original numerical defaults.
- [x] Produce patch from pristine baseline; apply in isolated replay, compare complete content and modes, and verify ZIP/TAR extraction equality and no ELF/cache/build/logs.
- [x] Save all six required deliverables and provide links; state real Arm/Takeoff/nonzero NOT AUTHORIZED and all missing integration evidence UNKNOWN.

Source archive captures the completed implementation/review checklist. Final artifact hashes, replay results and the external save operation are reported outside the immutable archive in PACKAGE_VERIFICATION.txt and the delivery response.
