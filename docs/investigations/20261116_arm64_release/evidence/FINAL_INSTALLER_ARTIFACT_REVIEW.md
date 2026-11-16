# Final installer artifact review

PASS. Read-only review of `dist/arm64/install_fc01.sh` and `dist/arm64/DELIVERY.json` on2026-11-16; no installer execution, service operation, VM operation or repository edit.

- `bash -n dist/arm64/install_fc01.sh`: PASS.
- Extracted the quoted heredoc Python body and called Python `compile(..., 'exec')` without executing it: PASS.
- Embedded Python equals `git show 56424d7:navigation_prebuilt_install.py` byte-for-byte after exactly one replacement of `parser.add_argument('--expected-sha256')` with fixed default and single allowed SHA256 choice. No other source change.
- Actual archive size6497136 bytes and SHA256 **3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123** match DELIVERY.json and both installer-pinned argument values.
- Actual installer SHA256 **7b35de4b7e498d46c56ac7b9abbf032c2ba64d1a76ab11177f9391bc4fc4e956** matches DELIVERY.json.
- DELIVERY source_commit resolves to56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a and control_candidate_commit toadc55d0b7c67e80bf9d77b1082208c898f95b756. DELIVERY reports4974 files; archive manifest/ELF/source inspection belongs to the parallel artifact reviewer.

Runtime dependency selection: shell launcher invokes only `/usr/bin/python3` and has no absolute oldSDK/build prefix. Embedded `dependencies()` clears AMENT_PREFIX_PATH, COLCON_PREFIX_PATH, CMAKE_PREFIX_PATH, PYTHONPATH, LD_LIBRARY_PATH and ROS_PACKAGE_PATH before sourcing `/opt/ros/humble/setup.bash` and candidate `$1/install/local_setup.bash`. Existing `FC01_PROJECT_ROOT` is recorded only as source metadata (`sdk_root`); it is not sourced or used to select runtime dependencies. Candidate service drop-in points ExecStart at candidate `prebuilt_boot.sh` and WorkingDirectory at the candidate entry. The script includes legitimate target paths `/dev/ttyS2` and system Humble/bin paths; no `/home/...`, `/tmp/...` or build-machine absolute prefix is hardcoded.

This review covers the shipped installer pin/source/syntax and its visible dependency-prefix selection. It does not claim actual installation acceptance or actual ARM runtime behavior; the separate VM and bundle reviews cover those.
