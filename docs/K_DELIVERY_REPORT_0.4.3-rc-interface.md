# Agent K — 0.4.3-rc-interface Delivery Report

Date: 2026-10-05

## Final decision

**PASS WITH CONDITIONS** — offline source candidate only.

**Real Arm / Takeoff / nonzero thrust / PX4 parameter writes are NOT AUTHORIZED.** This task did not connect an FC01, did not start a real MicroXRCEAgent, did not modify the user's real `current`, did not write PX4 parameters, and did not publish a real nonzero setpoint.

The implementation goal is satisfied: the Companion-side RC-link/source readiness no longer depends on `InputRc`; it now derives from the actually exposed `ManualControlSetpoint`, while `ManualControlSwitches` remains an independent, fail-closed RC_TAKEOVER requirement. The candidate preserves the 0.4.2 authority split, one-way takeover latch, ACK lifecycle, four flight gates, stage-one exact-zero behavior, final publication boundary, socket/update/install protections, and default-false/sentinel flight configuration.

The conditions are material: this execution environment has no ROS2 Humble/colcon, so a real Humble build/link against the exact generated pinned `px4_msgs` is UNKNOWN; and FIELD-EVIDENCE says `/fmu/out/manual_control_switches` currently has publisher count 0 on FC01, so **RC_TAKEOVER remains correctly unavailable on the real vehicle until that independent switch telemetry exists and is validated**. AUTONOMOUS_NO_RC is intentionally not coupled to RC telemetry, but its separate `autonomous_failsafe_confirmed` default remains false.

## Required inputs actually used

The actual user-provided/persisted 0.4.2 delivery was recovered and used, not reconstructed from chat summary:

- `I_DELIVERY_REPORT_0.4.2-authority-modes.md`
  - SHA256 `4451e1b7249c442fb620ae4767a67d47b8769280cc837212647197b8b68e3fb1`
- `J_PX4_V1_16_AUTHORITY_FAILSAFE_REVIEW.md`
  - SHA256 `67b6059d21d42fe91bbb3d3d4034a02cde38e4e74ef858822e883e6a21cfefab`
- `fc01_system_v0.4.2-authority-modes_source.zip`
  - SHA256 `bf0577e1a5b6969978e4fe59c4d14ef1ba8744743d5094be05ef3f30d12c4fba`
- `fc01_system_v0.4.2-authority-modes_vs_0.4.1.patch`
  - SHA256 `2d768fe0767bbccb25b76e473f9c838ef74757a3559c3d3dc5500d01de92f673`

The 0.4.2 source ZIP was unpacked and directly reviewed. Its active RC data path was:

`SystemNode /fmu/out/input_rc subscription`
→ `AuthorityTelemetry::rc(InputRc)`
→ timestamp/timestamp_last_signal/source/channel/lost/failsafe checks
→ `Facts.rc_link_ready`
→ `attitude::Feedback.rc_link_ready`
→ `authority::Inputs`
→ Core/Runtime route policy
→ final Arm/nonzero publication boundary.

`ManualControlSwitches` was already separate:

`/fmu/out/manual_control_switches`
→ `AuthorityTelemetry::switches()`
→ raw timestamp progression + Kill retention + switch generation
→ `rc_switches_ready`, `kill`, `operator_switch_generation`
→ takeover/release/final boundary.

## FIELD-EVIDENCE accepted from the task handoff

These are real-airframe facts supplied by the user and are kept separate from K's offline evidence:

- 0.4.2 built successfully on the user's Ubuntu/ROS2 Humble machine (`px4_msgs` PASS, `jikl_fc01_system` PASS).
- `/fmu/out/input_rc`: publisher count 0, Companion subscription count 1.
- `/fmu/out/manual_control_switches`: publisher count 0, Companion subscription count 1.
- `/fmu/out/manual_control_setpoint`: publisher count 1 from PX4 DDS.
- Observed `ManualControlSetpoint`: `valid=true`, `data_source=1`, centered sticks, throttle -1, `sticks_moving=false`.
- PX4 NSH has internal `input_rc`, `manual_control_switches`, and `manual_control_setpoint` uORB topics.
- SBUS works; `rc_lost=false`, `rc_failsafe=false`; CH5 Arm, CH6 Flight Mode, CH7 Kill; observed Kill `3 -> 1` and Arm `3 -> 1`.
- 0.4.2 observed `manual_signal=true`, `rc_link_ready=false`, `rc_takeover_ready=false`, consistent with the missing DDS publishers required by the 0.4.2 adapter.

K did not re-label any of those field facts as offline proof.

## Pinned schema verification

The candidate remains pinned to `px4_msgs` commit:

`392e831c1f659429ca83902e66820d7094591410`

The immutable commit's real `ManualControlSetpoint.msg` was checked directly, not inferred from PX4 main. Relevant schema:

- `MESSAGE_VERSION = 0`
- `timestamp`
- `timestamp_sample`
- `valid`
- `SOURCE_UNKNOWN = 0`
- `SOURCE_RC = 1`
- MAVLink sources 2..7
- `data_source`
- stick/aux fields
- `sticks_moving`
- `buttons`

The same pinned `ManualControlSwitches.msg` was checked directly. Relevant schema:

- `timestamp`, `timestamp_sample`
- `SWITCH_POS_NONE=0`, `ON=1`, `MIDDLE=2`, `OFF=3`
- `mode_slot`, `arm_switch`, `kill_switch`, other switch fields
- `switch_changes`

The actual ROS node now has a compile-time `ManualControlSetpoint::MESSAGE_VERSION==0` assertion in addition to the pre-existing pinned message assertions.

## InputRc -> ManualControlSetpoint semantic migration

### Old 0.4.2 RC-link predicate

0.4.2 required a unique `/fmu/out/input_rc` publisher and derived RC readiness from `InputRc` transport/source facts (`timestamp_last_signal`, `channel_count`, `input_source`, `rc_lost`, `rc_failsafe`) plus the existing PX4 `manual_control_signal_lost` failsafe flag.

That cannot become true on the current FC01 DDS graph because `/fmu/out/input_rc` has publisher count 0.

### New 0.4.3 RC-link/source predicate

0.4.3 subscribes instead to:

`/fmu/out/manual_control_setpoint`

and defines `rc_link_ready` as a conservative statement:

> PX4 is currently publishing one fresh, valid, selected RC manual-control source with advancing raw sample time, while the PX4 manual-control-loss flag is fresh and not lost.

Specifically, readiness requires all of the following:

1. exactly one `ManualControlSetpoint` DDS publisher;
2. fresh message envelope timestamp;
3. fresh raw `timestamp_sample`;
4. at least two accepted advancing raw samples inside the existing `flags_timeout` window;
5. `timestamp_sample <= timestamp` and envelope-to-raw age below timeout;
6. `valid == true`;
7. `data_source == SOURCE_RC`;
8. fresh PX4 failsafe flags;
9. `manual_control_signal_lost == false`.

A fresh envelope with duplicate `timestamp_sample` does not refresh RC activity. The first raw sample after startup does not establish sustained activity. After a stale gap, the first new raw sample also does not immediately restore readiness; the second fresh advancing sample is required. Invalid or non-RC selected sources fail closed.

This deliberately uses the selected/manual source chosen by PX4 rather than inferring physical RC from a raw transport source ID.

## ManualControlSwitches remains independent and mandatory for RC_TAKEOVER

0.4.3 **does not remove or bypass** `ManualControlSwitches`.

Its independent path still provides:

- raw `timestamp_sample` activity;
- `mode_slot`;
- `arm_switch`;
- `kill_switch`;
- other tracked switches;
- `switch_changes`;
- switch-generation change detection;
- Kill ON retention until a newer, fresh, mapped OFF sample.

`rc_switches_ready` still requires the unique switch DDS source, two progressing raw samples/freshness, and mapped Kill OFF. Missing publisher, stale data, first frame, duplicate raw sample, unknown Kill, or Kill ON fail closed. Therefore `rc_link_ready=true` alone cannot make `rc_takeover_ready=true`.

This is especially important on the current real FC01 graph: the task's FIELD-EVIDENCE reports `/fmu/out/manual_control_switches` publisher count 0. The new RC link adapter can now correctly recognize the selected RC source, but RC_TAKEOVER must still remain blocked until independent switch telemetry is actually exposed/validated. K did not weaken that contract to “make it run.”

## sticks_moving decision

`ManualControlSetpoint.sticks_moving` is implemented as **diagnostic-only evidence**, not a new takeover latch trigger.

Reason:

- PX4 v1.16 computes the bit from selected-source roll/pitch/yaw/throttle changes against the `COM_RC_STICK_OV` threshold.
- It demonstrates fresh selected-source stick activity, but does not by itself prove that PX4 executed an Offboard takeover.
- J's v1.16 review separately shows that actual Offboard stick override depends on `COM_RC_OVERRIDE` policy; the currently observed value `1` does not enable the Offboard override bit.
- Treating `sticks_moving` as a standalone latch would duplicate policy and could create false operator-release semantics without proving the intended takeover configuration.

So `rc_sticks_moving` is emitted only when the message is fresh, valid, selected `SOURCE_RC`, and the PX4 manual-loss flag is healthy. It does not satisfy switch readiness, confirmation, Arm, or nonzero-thrust gates, and it does not clear Kill. Existing takeover paths remain authoritative: switch generation, explicit companion takeover, PX4 `failsafe_and_user_took_over`, Kill/termination, PX4 failsafe, established Offboard signal loss, nav-state departure, and RC/switch telemetry loss during RC flight.

## AUTONOMOUS_NO_RC isolation

The autonomous route was not coupled to the new RC adapter.

With RC telemetry absent:

- AUTONOMOUS_NO_RC can still be selected/prepared if its normal health/safe-ground requirements are met;
- it still independently requires `autonomous_failsafe_confirmed=true` before authority readiness;
- the delivery default remains `autonomous_failsafe_confirmed=false`;
- no PX4 parameters are written or implicitly confirmed.

Tests explicitly cover autonomous preparation with all RC telemetry absent.

## JSON diagnostics added

Authority JSON now includes independent observability for:

- `rc_link_ready`
- `rc_source_seen`
- `rc_source_valid`
- `rc_source_selected`
- `rc_data_source`
- `rc_sticks_moving`
- `rc_switches_seen`
- `rc_switches_fresh`
- `rc_switches_ready`
- `rc_mode_slot`
- `rc_arm_switch`
- `rc_kill_switch`
- `rc_kill_latched`
- existing `rc_takeover_ready`
- existing `rc_takeover_confirmed`
- existing autonomous confirmation/readiness
- existing one-way takeover latch/reason.

This lets field debugging distinguish “selected RC source is healthy” from “switch DDS missing/stale,” Kill state, and route confirmation state instead of collapsing them into one false boolean.

## Implementation file changes

Primary implementation changes:

- `VERSION`
- `README_zh.md`
- `src/jikl_fc01_system/package.xml`
- `src/jikl_fc01_system/src/fc01_system_node.cpp`
- `src/jikl_fc01_system/include/jikl_fc01_system/authority_telemetry.hpp`
- `src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp` — diagnostic fields only; state-machine policy unchanged
- `src/jikl_fc01_system/include/jikl_fc01_system/runtime.hpp` — JSON diagnostics only; ACK transaction logic unchanged

Test/evidence changes:

- `tests/adapter_compile_check.py`
- `tests/test_authority_review.cpp`
- `tests/test_authority_telemetry.cpp`
- `tests/test_rc_interface.cpp` (new)
- `tests/test_authority_tools.py`
- `tests/authority_harness.cpp` (release label only)
- `tests/test_tools.py` (release/version expectations only)
- `docs/K_PINNED_SCHEMA_REVIEW.md` (new)
- `docs/K_ACCEPTANCE_RESULTS.md` (new)
- `docs/k_evidence/K_BASELINE_TEST_OUTPUT.txt` (new)
- `docs/k_evidence/K_CANDIDATE_TEST_OUTPUT.txt` (new)
- `docs/K_DELIVERY_REPORT_0.4.3-rc-interface.md` (this report, copied into source)

## Protected invariants retained

No functional changes were made to:

- `authority.hpp` route evaluation logic;
- `publication_guard.hpp` final authority/gate/health checks;
- `publication_boundary.hpp` real sink boundary;
- command 176/400 correlation semantics;
- ACK target/timestamp lifecycle;
- stage-one exact-zero enforcement;
- four flight gates;
- socket lease/server safety implementation;
- update/install activation semantics;
- configuration gate defaults;
- flight-parameter sentinel values.

The runtime file changed only to add diagnostic JSON fields; its `ack()` transaction implementation remains byte-for-byte identical within that function. The authority Core file changed only by adding diagnostic observation fields to `Feedback`; route policy and takeover state-machine logic were not changed.

The delivery still defaults to:

- `flight_actions_enabled=false`
- `arm_enabled=false`
- `takeoff_enabled=false`
- `flight_parameters_confirmed=false`
- `rc_takeover_confirmed=false`
- `autonomous_failsafe_confirmed=false`
- existing flight numerical parameters as sentinel/unconfirmed values.

## Offline test matrix

All required K items are mapped in `docs/K_ACCEPTANCE_RESULTS.md`.

Key results:

- pristine 0.4.2 `bash scripts/test.sh`: PASS;
- candidate 0.4.3 `bash scripts/test.sh`: PASS;
- all existing ACK/runtime/control/publication/socket/tool tests remain green;
- new ManualControlSetpoint tests cover invalid/non-RC/first-frame/second-frame/duplicate/stale/stale-recovery;
- independent switch missing/stale/unknown/Kill behavior remains fail closed;
- Kill retention and switch-generation one-way takeover are preserved;
- AUTONOMOUS_NO_RC remains independent from RC telemetry;
- JSON diagnostics are regression-tested;
- adapter syntax fixture was updated to the exact pinned ManualControlSetpoint field/constant shape and passes.

## Execution environment and UNKNOWN

K ran in:

- Debian 13
- g++ 14.2.0
- Python 3.13.5
- `ros2`: absent
- `colcon`: absent

Therefore the following remain **UNKNOWN** and are not represented as PASS:

1. real Ubuntu 22.04 / ROS2 Humble build/link of 0.4.3;
2. integration against the exact generated C++ types from the user's pinned `px4_msgs` checkout, beyond schema/API fixture validation;
3. real FC01 DDS timing/jitter/duplicate behavior for `ManualControlSetpoint`;
4. whether FC01 firmware can expose `/fmu/out/manual_control_switches` through DDS without a firmware/topic configuration change;
5. real RC mapping/calibration/failsafe ergonomics and physical pilot takeover;
6. real Kill behavior props-off and in armed state;
7. AUTONOMOUS_NO_RC physical failsafe behavior;
8. any armed/nonzero-thrust behavior.

The current field fact that `/fmu/out/manual_control_switches` has no DDS publisher is not UNKNOWN; it is a known blocker to RC_TAKEOVER readiness by design.

## Safety statement

This delivery is a source candidate and offline verification package only.

**It does not authorize real Arm, Takeoff, flight, nonzero thrust, PX4 parameter writes, or switching the user's real `current`.**

Before any real-airframe progression, at minimum the candidate must be built on the user's Ubuntu 22.04/ROS2 Humble environment with the exact pinned `px4_msgs`, its actual DDS graph must be re-checked, and the independent `ManualControlSwitches` requirement must have a real publisher and props-off validation for RC_TAKEOVER. AUTONOMOUS_NO_RC must remain blocked unless its separate failsafe contract is explicitly validated and confirmed.
