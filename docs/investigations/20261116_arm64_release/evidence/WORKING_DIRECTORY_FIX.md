# Real systemd WorkingDirectory correction

Fixed only navigation_prebuilt_install.py and tests/test_navigation_prebuilt_install.py. No commits/host systemctl/hardware.

Confirmed VM root cause: systemd249 WorkingDirectory is a scalar path directive, unlike ExecStart quoted argv token. Putting literal double quotes around path makes systemd parse a relative path beginning with quote; verify refuses before service stop.

Added separate systemd_directory serializer: absolute path required, reject controls/edge whitespace/backslash, percent escaped as%%, no added double quotes. ExecStart continues original systemd_quote token serialization.

Meaningful regression uses actual host systemd-analyze verify (Ubuntu22.04 parser) on temporary inert unit, absolute path includes spaces and% to exercise scalar andspecifier escaping. No service starts, root not required. Verify returns0. Install transaction test checks actual drop-in WorkingDirectory exact unquoted scalar. Ambiguous path rejection also covered.

`python3 -m unittest tests/test_navigation_prebuilt_install.py tests/test_navigation_prebuilt_service.py -q`:39 PASS (33 installer+6 service),0.58s. Log /tmp/fc01-installer-workingdir-tests.log. py_compile and gitdiff--check PASS.

Ready root commit/repackage and real VM re-run.
