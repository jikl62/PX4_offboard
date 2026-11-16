# Measured startup acceptance deadline

Changed only navigation_prebuilt_install.py and tests/test_navigation_prebuilt_install.py; no commits.

Actual ARM64 VM install6 candidate startup could not finish all immutable5000payload hashes/26ldd/type support checks within original installer30sec. Standalone same prebuiltcheck walltime46.377sec inTCG. Installer safely restored original exe/config/DDS, confirming failure recovery; this was installer acceptance budget, not controller stall/TTL.

Set named STARTUP_ACCEPTANCE_SECONDS=180 for verify_active only. No native/control/feedback/sourceTTL changes; grounded/current source evidence still required after startup. stop_and_wait/original restore deadlines unchanged (old restart measured13sec). No infinite retry.

Deterministic regression advances mocked monotonic without wallwaiting: no controller until40sec then actual existing version/config/safe ground validates. Forever no controller times out exactly180sec. Existing stale feedback and snapshot gate regressions remain unchanged.

41 combined tests PASS (35 installer+6 service),.51sec. /tmp/fc01-installer-startup-time-tests.log. py_compile/diff-check PASS. Changes include pending earlier snapshotorderfix; root can commit/repackage both.
