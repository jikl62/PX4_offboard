# Confirmed E2BIG installer fix

Fixed only navigation_prebuilt_install.py and tests/test_navigation_prebuilt_install.py; no commits.

Real VM reproduction:4974-entry manifest serialized as one process argument exceeded Linux MAX_ARG_STRLEN, causing OSError E2BIG before service stop. dependencies() parsed that JSON but never used it.

Minimal correction: remove manifest JSON argument and unused parsing/imports. Dependency child takes only release path, still sources pinned Humble+bundle, verifies required ROS type support, scans all payload ELF and runs ldd. Existing earlier verify_tree already checks full manifest. No protection removed or changed service state.

New regression constructs 5000-entry realistic manifest well above128KiB, captures actual Backend command, proves all args below128KiB and manifest absent. Parses generated nested Python with ast and confirms only sys.argv[1] path referenced; script remains syntactically valid.

`python3 -m unittest tests/test_navigation_prebuilt_install.py tests/test_navigation_prebuilt_service.py -q`:37 PASS (31 installer+6 service). `/tmp/fc01-installer-e2big-tests.log`.
`python3 -m py_compile navigation_prebuilt_install.py`, `git diff --check`:PASS.

Ready for root commit/repackage and VM re-run. No production control code changed.
