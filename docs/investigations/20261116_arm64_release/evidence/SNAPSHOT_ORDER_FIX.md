# Snapshot read order correction

Changed only navigation_prebuilt_install.py and tests/test_navigation_prebuilt_install.py; no commits.

Real ARM64 TCG VM measured systemctl show cost .10-.13s. Old live_state read1Hz snapshot BEFORE that slow query and then validated original .25s TTL, needlessly consuming most freshness window. Reordered each loop: query actual service identity, read process starttime/boot identity, then read newest atomic snapshot and immediately validate original native monotonic time plus source TTLs. All identity bindings retained. No TTL/gate relaxation, no timestamp rewriting.

New direct Backend test simulates .13s query during which Supervisor produces newer actual state: proves command order servicequery then snapshotread and accepts new original timestamp. Separate assertion same old timestamp remains rejected; cannot freshen stale data. Existing PIDreuse/bootstale/sourceexpiry tests remain.

40 combined tests PASS (34 installer +6 service), .53s. Log /tmp/fc01-installer-snapshot-order-tests.log. py_compile/diff-check PASS. Ready root commit/repackage.
