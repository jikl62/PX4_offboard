# Orange Pi FC01 Control Implementation Plan

> For agentic workers: execute inline with superpowers:executing-plans, then one independent whole-change review.

Goal: one-file offline deployment and live console for existing ARM64 takeoff, altitude hold and landing.
Architecture: preserve the C++ controller, add session configuration, ownership-aware process supervisor and a sequenced Unix-console client.
Tech Stack: Python 3.10+, Bash, PyYAML already present with ROS2 Humble, existing R6.3.2 native install.
Spec: docs/superpowers/specs/2026-10-27-orangepi-control-design.md

## Global Constraints

- Original P0 source/install/config and PX4 parameters remain unmodified.
- Default check has closed action gates; --flight plus interactive takeoff is required for motion.
- Preserve numeric flight limits and all original C++ guards; retain RC_TAKEOVER, 115200 and domain 0.
- Reused Agent is never terminated. No automatic Arm retry or force-kill of the controller.
- Do not equate host tests, local publish success, or bench observations with real hover validation.

## Review Focus

- Stale or out-of-order replies must not advance a takeoff plan.
- Cancellation during preparation must never submit TAKEOFF later.
- An operator quit/EOF/SIGINT while airborne must preserve the recovery channel.
- Existing UART owners/controllers and pre-existing paths must be preserved.
- Installer paths, archive entries and report contents must not expose credentials or overwrite unrelated files.

### Task 1: session configuration and Mission sequencing

Files: fc01_control.py, tests/test_control.py.
Interfaces: runtime_profile(base, flight), Mission.request_takeoff/observe/poll/sent/reply/cancel.
- [x] Write behavioral tests and observe RED before implementation.
- [x] Implement copied configuration and explicit live takeoff sequencing.
- [x] Verify no launch-time motion, stale/invalid state refusal, no duplicate takeoff and cancellation.
- [x] Commit with test result in ledger.

### Task 2: launcher, Unix client, lifecycle and report

Files: fc01_control.py, start.sh, tests/test_control.py.
Consumes: Task 1 configuration and Mission API.
Produces: --check/--flight CLI, report.zip, reusable Agent/controller ownership rules.
- [x] Write protocol/ownership/lifecycle/report tests and observe RED.
- [x] Implement environment loading, verified native executable, process supervision, live console and automatic evidence.
- [x] Run whole suite and original C++ authority/ACK regressions.
- [x] Commit with evidence.

### Task 3: single-file delivery and review

Files: build_bundle.py, README_zh.md, VERIFICATION.md, tests/test_installer.py.
- [x] Test offline extraction, tamper refusal and preservation of existing files, observe RED.
- [x] Build the self-extracting .sh and verify install-only path.
- [x] Run complete suite; request one independent code review; fix important findings with regression tests.
- [x] Save final delivery persistently and provide one invocation plus takeoff/land commands.
