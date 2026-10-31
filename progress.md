# SDD ledger — plan: docs/superpowers/plans/2026-10-27-orangepi-control.md

Execution: native, user opted out of repeated design questions and asked to finish implementation.
Workspace: new isolated git repository work/orangepi-hover; archived reference sources remain outside it.
Pre-flight: Task 2 consumes Task 1 Mission and runtime_profile; Task 3 packages the tested Task 2 entry. API contracts are defined in the plan and tests.
Workflow adaptation: cloud skill helper scripts are not local executables; use native git, this ledger and explicit verification commands for bookkeeping.
Known limitation: no direct access to the user's Orange Pi or flight hardware; native ARM64/ROS2/real hover verification cannot be claimed by this environment.

Task 1 complete: wrote 15 behavioral tests first and observed all fail because the entry did not exist. Implemented copied runtime configuration and Mission transaction sequencing. `python3 -m unittest discover -s tests -v`: 15 passed (2026-10-27 UTC). No hardware flight performed.

Task 2 complete: protocol/ownership/report/lifecycle tests observed RED before implementation. Found and regression-tested final-state loss at EOF; fixed queued-byte draining. Further restricted owned-Agent shutdown to a clean native exit plus a fresh IDLE/SAFE_DISARMED ground state. 28 Python tests passed. Real Unix stream socketpair + child-process fixture verifies explicit takeoff sequencing and heartbeat retention through quit/descent; named Unix socket creation is restricted on the host, so hardware/path binding is not claimed. Four unchanged original C++ regressions compiled with C++17, -Wall -Wextra -Werror -pedantic and passed: authority_runtime, authority_boundary, r632_skewed_176_ack, r632_prompt_176_ack. Output: /tmp/fc01_original_regressions.log. start.sh loads installed Humble/px4_msgs; all preflight errors route to the automatic report.

Task 3 candidate: 8 installer tests failed RED for the missing self-extractor, then complete suite passed 36 tests. Verified SHA/CRC failure, archive traversal/symlink refusal, no overwrite of unequal files, equal reinstall, and preservation of the original operator stdin across heredoc unpacking. Python 3.10 syntax, Bash syntax and deterministic bundle build passed. Independent whole-change read-only review follows, as required by the execution/review skills; no implementation subagents were used.

Whole-change review complete (one fresh-context reviewer, ae9c3d7..e0c0ecf). Both Important findings reproduced in three RED regressions: broken stdout escaped recovery; blocked stdout stalled it; slow report compression lost heartbeats. Fixed with bounded background display and ZIP deferral until native exit. Added pending-Arm quit/EOF/signal integration subcases. Complete final suite: 40 tests passed in 11.814 s; diff check clean. Reviewer scope exclusions are explicitly accepted and documented in VERIFICATION.md. No second review seat or implementation agents were used. Final packaging/persistence follows.

Delivery complete: final self-extractor built, Python 3.10/Bash syntax and install-only extraction verified against exact launcher/config/README/verification bytes. 54,447 bytes, SHA256 0fdb42f2149844313ff1fd1daad9d31e45259924bfb5712ca1b9ab42e1020559. Saved as libfile_9330735214b881918a8a0fe06e123b7c; returned identity/xattrs applied successfully to the original local artifact. Local checkout completion bookkeeping occurs after packaging; runtime sources are unchanged. User invocation: bash ~/Desktop/FC01_OrangePi_Control_20261027.sh --flight; then takeoff 0.5 / land / quit. No hardware-flight verification claimed.

R2 investigation: device report showed native exit before socket. Found a deterministic wrapper defect: flight_enabled was enabled even though native Config::validate() prohibits it for attitude_altitude. Corrected the erroneous Python test expectation and added real native-configuration integration. Before product fix, native probe reproduced exit 2 with `attitude_altitude uses separate flight_actions_enabled gates`; closed-gate mode and intentional legacy-flag rejection passed. Product fix keeps flight_enabled false and enables ACTION_FLAGS[1:6]. No user log upload was required to reproduce this defect. Earlier host verification was insufficient at this specific generated-config/native-validation boundary.

R2 verification/delivery: 43 tests passed (12.444 s), including all three native-config contract tests; eight installer tests passed after R2 directory/version changes. Verified Python 3.10 grammar, Bash syntax, exact extraction and the one-time R1 repair against the actual R1 git revision: backup preserved, repeat repair idempotent, caller stdin retained, unknown version refused. Updated bundle is 59,272 bytes, SHA256 6b8dd9aeff3b1f720b1df4f3cefbd5de02a5e6caa8cfc8071ff5505562b4d546; replaced existing libfile_9330735214b881918a8a0fe06e123b7c and applied returned metadata successfully. Runtime sources numerics unchanged. Device confirmation of the corrected entry is still pending; no successful native ARM64 startup/flight claimed.


## 2026-10-27 closed-gate autostart delivery

# SDD ledger — plan: docs/superpowers/plans/2026-10-27-orangepi-autostart.md
Pre-flight: no shared task interfaces (one task). Existing standalone scratch repository and feature branch isolate the work from the original native installation. User says start now and has already asked to reduce confirmations; no further design gate requested.
Baseline: 43 tests; one initial timing failure test_broken_stdout_cannot_abandon_native_recovery (no TAKEOFF). Separate integration suite rerun 6/6 passed; preserve and investigate if reproduced, do not claim the first run was green.
Task 1: in progress, base 138cb8529fb4dcebd6bf644e3b7a04ba33f6c272.
Task 1: complete (commits 138cb85..2a6716e, tests: python3 -m unittest discover -s tests -v → OK)
Final review: fresh gpt-6-astra (read-only) found one Critical and three Important defects; no minors. Review range 9aff49b..2a6716e. Implementer performed the single fix pass.
Final: fixed invalid ROS CSV override — test_standby_overrides_source_csv_file_path_with_yaml_empty_string and effective native fixture override parsing RED→GREEN; suite 63/63.
Final: fixed retained-Agent restart blockage — generated-unit policy RED→GREEN, real owned Agent stop/retain/start/reuse passes, surviving native preflight refusal passes; suite 63/63.
Final: fixed external ROS file logger bypass — native launch effective logging contract RED→GREEN; --disable-external-lib-logs; suite 63/63.
Final: fixed stale dependency wait snapshots — midpoint status check RED→GREEN, one preflight per 30 seconds retained; suite 63/63.
Final: Ruling: use KillMode=process with no SIGKILL — supervisor owns native graceful stop, preserved Agents can be reused, and process scan refuses surviving native; cost: a surviving controller after supervisor crash needs resolution before a new controller can start.
Final: Ruling: CH5/CH6 navigation task remains deferred — user explicitly requested boot script first; cost: selecting CH6 does not initiate a mission in this package.
Final: Ruling: Pi reboot/UART/DDS checks remain device-only — no remote device connection; host tests cannot certify them; cost: device environment differences may require the supplied report.
Final: Ruling: VERTICAL_VELOCITY_INCONSISTENT remains unresolved and unchanged — outside startup scope; cost: this package establishes standby only, not validated autonomous hover.

Final bundle: dist/FC01_OrangePi_Autostart_20261027.sh, 62,288 bytes. SHA256 31c53f49b11fd9e6f6fdb2d2831046d53daf740b8978b6f71e6a31d76a6c8bfa. Final artifact extracted and compared: 17/17 files; installed manager --help passes. Saved as libfile_244a46de9ccc81919b1091776f99e14f, file_00000000ffa881f896867bd05cdb8024; metadata helper succeeded. Full final suite 63/63 in 17.433s; Python 3.10/Bash syntax passed. Kept standalone work/orangepi-hover branch; no remote integration requested. Pi install/reboot remains user-device operation.
