# RC Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a complete boot-time native controller that observes CH5 manual arm, CH6 navigation entry and executes guarded .5 m relative-height flight.

**Architecture:** Default-false native RC startup phases join the existing proven attitude/vertical flight/recovery phases. A Python supervisor supplies process/Agent/log/status management only; an offline installer builds a private native overlay and upgrades the managed service. A terminal monitor is read-only and independent of control.

**Tech Stack:** C++17, pinned PX4 1.16 px4_msgs, existing Humble rclcpp/rclpy, Python 3.10, Bash, CMake, systemd 249, XDG desktop entries.

**Spec:** docs/superpowers/specs/2026-10-28-rc-navigation-design.md

## Global Constraints

- No automatic ARM or Offboard-entry command in the RC route; CH5 owns manual arm, CH6 mode slot 6 owns navigation entry. `arm_enabled=false` throughout this route.
- .5 m default target, 20 Hz, original 30-second task cap and original numeric/freshness/feedback/tilt protections unchanged; no new flight-calibration claim, XY/GPS requirement or vision input.
- New process requires disarmed/landed/resting Stabilized, CH5 OFF, CH6 non-navigation, fresh RC/clock/exclusive sources, actual prestream commits and PX4 signal evidence; held controls/armed boot cannot launch.
- Shared lock and process scan, ownership-aware Agent retention, no SIGKILL; UI and logs cannot own the control lease. UART /dev/ttyS2 115200, domain 0.
- Private native overlay and build/install prefix; preserve installed candidate and old services/files. Offline one-command native build (one job), no dependency downloads.
- Native stdout 4 MiB×3, supervisor 1 MiB×3; CSV YAML empty string, ROS external logger disabled; state updates <=5 seconds in retries too.

## Review Focus

- Delayed/reordered RC arm switch, armed status, mode selection and mode status must not bypass fresh independent authorization; no early nonzero thrust or fabricated command ACK.
- Actual target/heartbeat publication may fail between sinks; readiness uses successful commits and final resampled context, never only a planned loop duration.
- RC mode exit, supervisor disappearance, pending landing/disarm and estimator faults must preserve original custody semantics and never reacquire Offboard or auto-rearm.
- Stale boot snapshots, already-armed vehicle or absent build tools must not cause unsafe service replacement, enable two controllers, or silently claim ready.
- Closing/reopening desktop monitor, full logs, nonstandard paths and an Agent retained across stop/start must not alter command ownership or block recovery.

### Task 1: Native RC admission, final boundary and flight integration

**Files:** Create `native/` snapshot of supplied native package/tests/config/tools, baseline hashes, `native/tests/test_rc_navigation.cpp`, `tests/test_navigation_native.py`, `native/src/jikl_fc01_system/include/jikl_fc01_system/ros_config.hpp`, native config-check source. Modify native Core, runtime, authority telemetry, ROS parameter mapping, publication guard and node in overlay only.

**Interfaces:**
- Consumes unchanged vertical controller, feedback guard and original C++ regression fixtures.
- Produces Config fields `rc_navigation_enabled`, `rc_target_height`, `rc_navigation_mode_slot`, `rc_arm_to_navigation_timeout`; Core RC phases and `note_rc_publication(bool,double)`; Runtime `confirm_rc_output(bool,double)` and RC status JSON; actual YAML-consumer config check executable.
- Runtime Feedback carries raw RC axes/switch sample stamp, armed_time and latest_arming_reason from accepted sources. Final ARM policy rejects RC navigation independently of upstream output.

- [ ] Write native regression scenarios for boot-held/armed, stale/unsafe sources, no actual commits, correct manual-arm/slot6 path, opposite source arrival order, wrong arming reason, premature/held nav, pre-arm rebase vs armed freeze, final-sink races and zero-only staging. Observe RED against pristine overlay.
- [ ] Implement opt-in RC startup without weakening default behavior, join existing ArmedGround/flight/recovery phases, gate all console arm/takeoff entry points, preserve mode-out handover and landing/disarm. Add real commit callback and native config-check executable sharing parameter declaration with actual node.
- [ ] Run `python3 -m unittest discover -s tests -p test_navigation_native.py -v` and original native C++ suite plus adapter syntax fixture; expected PASS including all legacy defaults. Run config contract for RC gate combinations; expected manual-only valid and automatic-arm/bench invalid.
- [ ] Commit native unit; ledger baseline/range/tests. No hardware claims.

### Task 2: Background controller, state view and management

**Files:** Create `navigation_service.py`, `navigation.py`, `navigation.sh`, `navigation_boot.sh`, `tests/test_navigation_service.py`. Modify shared standby supervisor only through backward-compatible keyword options and status metadata.

**Interfaces:**
- Consumes native executable at `<entry>/install/lib/jikl_fc01_system/fc01_system_node`, config check executable alongside it and Task 1 RC YAML parameter names.
- Produces `navigation_profile(base)` with explicit manual-only gates, daemon validation and supervision; management CLI actions install/status/monitor/logs/report/start/stop/land/disable/rollback; readable state in `~/FC01_navigation/status.json` including phase/reason/current mode/actual armed/relative height and native heartbeat evidence.

- [ ] Write tests: real socketpair/process READY->manual arm->nav->flight state trace, supervisor SIGTERM keeps native recovery until safe exit, no console INTERACTIVE/PING/TAKEOFF, never stops reused Agent, native override marker check, bounded status/logs, live read-only monitor and stale/fault display.
- [ ] Implement configuration/environment/management and read-only desktop-login monitor. UI never sends setpoints, heartbeats or arm; graceful stop/land uses supervisor/native shutdown.
- [ ] Run new tests and complete Python suite; expected PASS, no regression of the 63-test standby/interactive baseline. Commit and ledger.

### Task 3: Offline build, safe upgrade, desktop entries and deliverable

**Files:** Create `build_navigation_bundle.py`, `navigation_install.py`, `README_NAVIGATION_zh.md`, `VERIFICATION_NAVIGATION.md`, installer tests and deployment probe script. Extend extractor limits only if actual curated payload exceeds original safe limits.

**Interfaces:**
- Consumes Tasks 1/2 overlay, native config-check, manager and service; builds with `/bin/bash` ROS/base environment + `cmake --build --parallel 1`, never colcon all or apt.
- Produces `dist/FC01_OrangePi_Navigation_20261028.sh`, one-command install and diagnostic ZIP on any build/probe/upgrade failure; managed `fc01-navigation.service`, desktop shortcut and XDG read-only monitor autostart.

- [ ] Write install behavior tests for integrity, exact extracted payload, missing compiler, safe native build commands, passive fresh-ground gating, unknown-unit refusal, old managed standby disable only after build/config/ground success, repeated install without disruptive restart, safe rollback and space/$/% paths.
- [ ] Implement build/check/probe/install sequence and scoped report. The passive probe observes source clock, uniqueness and new disarmed/landed messages; armed/unknown states stop installation before service mutation. Preserve/match Agents using existing ownership checks.
- [ ] Run all native and Python tests, actual adapter fixture, Python 3.10/Bash/unit/desktop checks and final artifact extraction; expected PASS. Record exact counts, limitations and hashes. Commit.
- [ ] One fresh-context whole-change review per execution skill; fix Critical/Important in one RED→GREEN pass and full suite, ledger all rulings. No second review.
- [ ] Persist the single verified source/installer artifact to Library and return one installation command, final operator sequence and explicit physical-test limitations.
