# Execution ledger — plan: docs/superpowers/plans/2026-10-04-attitude-altitude.md

Baseline: supplied r2 ZIP/TAR byte-identical (30 files); handoff SHA256 checks PASS; original ack lifecycle tests PASS.

Pre-flight: Task 1 produces quaternion/vertical/ownership core; Task 2 consumes them; Task 3 consumes Runtime outputs; Task 4 audits and packages the same source. Interfaces consistent.

Ruling: User explicitly supplied a reviewed specification/contract and directed continuous implementation; use that specification, do not add another design approval round. Cost if wrong: review of delivered source is still possible before installation.

Ruling: C report's continuously-followed yaw and manual-Arm entry are superseded by the formal fixed-local-yaw and stage-one armed-refusal contract. Cost if wrong: stage one rejects rather than permitting manual motor activity.

Ruling: 02's later true-machine ACK PASS supersedes A/B's older unverified statements. ACK handler/ground core remain unchanged. Vendor Commander byte identity remains UNKNOWN.

Ruling: Contract requires no automatic current switch; remove installer's baseline first-install symlink side effect. Cost if wrong: first activation needs an explicit update.sh invocation.

Environment: Ubuntu 24.04 x86_64; ROS2 Humble/colcon absent. Pure C++/script tests executable; real Humble and ARM64 build/runtime remain UNKNOWN.

Task 1 RED: original ACK PASS, new contract test assertion fails because attitude/vertical implementation is absent.
Task 1 Ruling: lower-clamp fixture resets prior thrust to .4 before expecting 0 with slew=10/s and dt=.05; prior .6 can only fall to .1 in that interval. Expected clamp is isolated while a separate test covers slew.

Task 1 complete: quaternion/vertical/ownership and original ACK suites PASS, commit 1c601e1.
Task 2 RED: assertion for absent attitude-altitude state machine. GREEN: full lifecycle, ACK/new-status conjunction, gates and fault injection PASS; original ACK suite PASS.
Task 3 RED: unsupported ready command, piped console accepted, wrong export prefix, single-command LAND acceptance. Native AF_UNIX tests encountered EPERM from execution sandbox.
Ruling: retain native socket tests but skip only permission-denied OS operations; test the actual CLI wait loop over an OS-PTY transport fixture and regular-file socket protection independently. Native socket runtime remains UNKNOWN; no sandbox override requested.

Task 2 complete: runtime and original ACK suites PASS, commit 625fb9f; added and proved a premature-height sample cannot skip the ascent target ramp.
Task 3 GREEN: CLI tests (8 tests, 7 PASS/1 native AF_UNIX SKIP), source-only export, first/existing install no-current side effects and explicit update/rollback fixtures PASS. Installer test was run RED against the supplied baseline and GREEN against this release.
ROS adapter translation unit: strict -std=c++17 -Wall -Wextra -Werror -pedantic -fsyntax-only PASS with isolated interface fixtures; this is a syntax/field mapping check, not real Humble compilation.
ACK handler and track(), ground_link.hpp, unix_server.hpp and velocity_core.hpp compared byte-for-byte with the supplied baseline: PASS.

Independent code review (read-only) requested changes: ground cancellation could add hover feed-forward; shutdown could accept queued new tasks; an open but hung interactive preparation console did not expire its PING lease. None bypassed the default flight lock.
Review fix 1 RED: cancelling from ARMED_GROUND failed thrust<=previous. GREEN: fresh confirmed ground cancellation uses a ramp-to-zero path, new land samples and dwell before serialized Disarm. LAND/disconnect/shutdown from ARMED_GROUND and early THRUST_RAMP are tested.
Review fix 2 RED: no Runtime shutdown admission latch/API existed. GREEN: begin_shutdown latches requests, queued preparation/interactive/takeoff/mode are rejected, status/PING/landing remain available; adapter checks stop flags before Arm publication.
Review fix 3 RED: PRESTREAM remained active after interactive PING timeout. GREEN: an established interactive lease expires during preparation and latches SAFETY; a one-shot ready session without INTERACTIVE remains supported and releases on disconnect.
All tests and the isolated-fixture strict adapter syntax check PASS after these fixes; native AF_UNIX tests still SKIP/UNKNOWN as above. Real ROS compilation is not implied.
Review follow-up RED: a valid slow-slew (.1/s, hover .4, ramp .5s) fixture reaches ASCEND before liftoff; cancellation still increased thrust. GREEN: ground cancellation now uses confirmed physical ground feedback independently of phase name, including ASCEND; all three cancellation mechanisms are tested for all three progress points (9 cases).
Independent final re-review at f10801c: approved for source delivery, with no outstanding Critical/Important/Minor findings. Hardware/build/transport limitations remain explicitly unverified.
Final executable verification: test.sh exit 0 (native AF_UNIX SKIPs reported), shell syntax 12 files, Python AST 5 files, isolated-fixture node syntax, ACK/track + 15 full-file byte invariants PASS. Actual install attempt rejected Ubuntu 24.04 before ROS build, leaving no release/current; full Humble build UNKNOWN.
Integration ruling: user explicitly chose independent tar.gz/zip/diff source delivery, not a merge or deployed service. Preserve the local implementation branch and pristine baseline; no extra integration-choice approval is applicable.

Source package first-pass verification: 58 files; tar/zip byte parity, extracted permissions, clean-r2 patch replay and standalone report parity PASS. Final frozen package and persistence are completed as the delivery operation; package checks are recorded externally in PACKAGE_VERIFICATION.txt.
