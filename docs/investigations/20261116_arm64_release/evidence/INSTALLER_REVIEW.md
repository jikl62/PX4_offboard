# Prebuilt installer final review including Agent handoff

2026-11-16 21:16. Read-only final review of navigation_prebuilt_install.py and tests/test_navigation_prebuilt_install.py after original review findings and owned-Agent follow-up. No reviewer source changes/commits/systemctl/hardware/Agent operations.

Result: no unresolved blocking code defect found in reviewed scope. Latest related tests PASS:36 total (30 installer+6 prebuilt service), plus py_compile and git diff --check. These tests inject the system manager/process/device backend and fake ELF files; they do not establish actual ARM64 loader, hardware UART, flight or real systemd lifecycle.

## Agent handoff contract checked

- Original ownership determined from actual Agent ancestry under service MainPID. Only owned Agent with existing Supervisor exact argv suffix `serial -D /dev/ttyS2 -b 115200 -v 0` qualifies for handoff. Unknown args reject before stop; no installer action kills or starts Agent.
- Existing shared Agent remains exact whole original process identity, including PID/uid/exe/SHA/argv. Disappearance or replacement refuses; never reclassified as owned handoff.
- Old service stops gracefully; no new config writes until zero controllers and inactive/failed service MainPID0. Only original-owned case may temporarily have no Agent, and only after real fuser returns1 with empty stdout AND stderr and /dev/ttyS2 is still a character device. Reviewer identified missing stderr condition; author fixed it and added direct Backend regression (not only Fixture override).
- Original Supervisor owns safe cleanup; new unchanged Supervisor can create same original agent executable with exact existing UART/baud/args. After startup verify_agent allows PID change only for original-owned case and still requires uid/exe/SHA/argv equivalence and real sole UART ownership. Surviving old-owned Agent is reused without duplicate creation.
- Rollback/recovery applies same ownership-aware check; saved original ownership retained in BACKUP.json, original settings/executable identity verified after restart. Repeat install leaves active Agent alone.

Tests include owned handoff install/repeat/rollback PID change with same executable/args, unknown owned args refusal before stop, shared disappearance refusal, real Backend UART empty proof vs diagnostics/missing character device, and altered actual policy mismatch.

## Earlier findings confirmed resolved

Full original review: /tmp/fc01-prebuilt-installer-independent-review.md. Single-client IPC conflict resolved by existing Supervisor atomic snapshot with boot/MainPID/controller/process-start/native timestamp checks and accumulated source TTL. Actual friendly READY/PRESTREAM statuses and prearm zero-output active/heartbeat now supported. Before switch/new acceptance/old recovery native-reported policy fields must equal runtime YAML; whole original config/DDS and old exe hash must verify after rollback/recovery. Inactive failed candidate can safely roll back without a running controller. Archive/payload hash/architecture/dependency checks all precede service stop. Dedicated transaction drop-in only; original unit/dropins/environment files preserved and external edits refuse.

Caveats carried to parent validation: only Ubuntu22.04 ARM64/Humble/domain0/UART2 115200 existing contract supported; no generic flight authorization or controls adjusted. Packaged C++ checker creates parameter-only ROS context but no flight publishers/spin/serial. Actual ARM64 install/ldd/typesupport and real PID1 VM lifecycle must be reported separately from these fixtures.
