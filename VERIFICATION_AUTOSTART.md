# Autostart verification — 2026-10-27 delivery

Scope: closed-gate native background standby, separate from RC flight mission implementation. No production C++ change, FC parameter change, automatic arming or Offboard preparation.

- Full host suite: `python3 -m unittest discover -s tests -v`, 60/60 passed in 15.069 s before final packaging.
- New tests: 10 standby and 7 systemd/installer/diagnostic tests, first observed RED before implementation. Real subprocess/socketpair exercise of STATUS-only supervision, signal stop, socket loss, Agent loss, lock contention, closed-gate native argv/config, fresh vs stale state, retry delay and bounded rotating files.
- Generated service accepted by `systemd-analyze verify`; custom path includes spaces, percent and dollar signs. Native binary/ROS environments validated only at installation on the Pi, not fabricated in host tests.
- Python 3.10 grammar and Bash entry syntax checked. Actual offline installer extracted twice with payload bytes compared.
- Native Config::validate() contract tests (existing suite) consume generated closed-gate config; no gate bypass introduced.
- Host syscall restrictions prohibit creating named Unix sockets and expose a different /proc PID namespace. The full supervisor test substitutes only the socket connection/process metadata boundary, uses a real socketpair and child process; it does not claim filesystem socket or UART validation on ARM64.
- Initial baseline had one existing timing-sensitive failure in `test_broken_stdout_cannot_abandon_native_recovery` (TAKEOFF not submitted); the isolated integration rerun was 6/6 and the later full 60-test run passed. Existing flight implementation was not changed.

Fresh independent review found four deployment/observability defects; each received a regression observed RED before correction: zero-length ROS CLI CSV override, retained Agent blocking mixed-mode systemd restart, external ROS files outside rotation, and waiting snapshots expiring during backoff. Final corrections force CSV to YAML `''` and omit the override, use intentional KillMode=process with supervisor-owned shutdown, pass --disable-external-lib-logs, and refresh status every 5 seconds while retaining the 30-second preflight interval. Added real owned-Agent retain/stop/start/reuse and surviving-native duplicate prevention scenarios. No second review dispatched; the final full suite verifies the fix pass.

Final fix-pass verification: full suite **63/63 PASS in 17.433 s**, including generated systemd unit verification. Python 3.10 grammar and Bash entry syntax also pass after the fixes. Existing flight code remains unchanged. The initial 60-test result above is retained as history, not substituted for this final run.

The Pi has not been accessed remotely. Enabling on that device, reboot behavior, serial device permissions and DDS reception remain device checks. Previous VERTICAL_VELOCITY_INCONSISTENT is unresolved and unchanged; this delivery does not authorize navigation flight. Retained Agent output is /dev/null, and reused Agent logs stay external. ROS parser semantics were checked against Humble upstream source; the host fixture rejects zero-length override input but is not the real rcl library.
