Current delivery: [20261110 recovery diagnostic V5 release](docs/FC01_RECOVERY_DIAGNOSTIC_V5_20261110.md), version `20261110.7`. V5 includes the landing output, vertical-only receive, ground proof continuity and sample expiry display fixes; the previous V4 package remains available as a historical artifact. `HOST_VERIFICATION.json` records the host checks for this package; target ARM64 evidence is generated only after native compilation on the Orange Pi. The record below describes the original release and its environment at that time.

# RC navigation verification record

Scope: native manual-RC admission integrated into the supplied R632 attitude/height controller, supervised boot service, visible read-only status and staged offline installation. The original supplied candidate tree remains unchanged. `native/BASELINE_SHA256.json` records the pristine overlay inputs.

## Evidence

- Before modification, 51 original native C++ regression programs passed.
- The host runner now executes 54 native C++ programs, including startup/ordering/final-boundary tests and a synthetic path through HeightHold, landing, actual ground-disarm publication and new-state confirmation. No automatic ARM or Offboard-entry command is permitted in the RC route.
- The actual node and actual configuration-check source compile against the pinned ROS/message API fixture. Actual generated RC YAML parameters are also validated against the real native Config header, including manual-only arming and mode-slot restrictions.
- Real child-process/socketpair tests verify supervisor STATUS-only management, recovery outliving a stop request, reuse of the existing Agent and status-instance/boot/freshness handling. Native logging tests exercise pipe backpressure and broken readers.
- Installer tests exercise passive source-clock/advancing/unique ground proof, armed-observation veto, subscription-only operation, build/config/probe sequencing before mutation, single-job offline staged build arguments, unmanaged-unit/drop-in/process/path refusal, repeat installation, rollback sequencing and paths containing spaces, percent signs and dollar signs.
- The actual self-extractor is executed twice and extracted files compared byte for byte. A differing existing file is preserved and rejects extraction. Integrity/path/symlink/bounds guards are reused from the separately tested original extractor.

Commands:

```bash
python3 -m unittest discover -s tests -v
python3 tests/run_native_cpp.py
python3 build_navigation_bundle.py dist/FC01_OrangePi_Navigation_20261028.sh
bash -n dist/FC01_OrangePi_Navigation_20261028.sh
```

The initial complete suite passed 92 tests. A fresh whole-change review found three Important issues (no Critical/Minor issues): delayed pre-ready authorization, configuration-only reinstall, and stale ground authorization across password entry. Each received a regression observed failing before its fix; all are included in the final complete suite. Readiness now fences source events using adapter time plus the existing100 ms future-source allowance. Prepared effective YAML participates in update detection. Administrative authentication precedes the final passive observation; subsequent mutations use noninteractive sudo and same-boot monotonic ground evidence bounded to2.5 s, with re-observation on expiry. Rollback shares the latter protection.

The final suite passed96 Python test cases, including all54 native C++ executables and actual-node/checker API fixture checks. Final extraction/rebuild/syntax checks are repeated after these review fixes.

## Limits

The host has g++ but lacks CMake, colcon, an ARM64 cross compiler and a live Humble installation. Host adapter checks use pinned API fixtures and cannot prove real Humble linkage or physical behavior. On-device installation must build the actual native package and invoke its real rclcpp YAML consumer before changing managed services. Missing dependencies produce a bounded diagnostic report; nothing is downloaded.

No new physical hover, thrust calibration, horizontal hold, vision integration or repair of the earlier VERTICAL_VELOCITY_INCONSISTENT event is claimed. Existing thresholds, limits and fault behavior remain in force. The installer is hardware-aware but has not been executed on the user's Pi from this environment.

Review rulings: physical acceptance remains unclaimed until device flight evidence exists; actual ARM64/Humble linkage is enforced by the on-device build gate rather than inferred from fixtures; exhaustive re-audit of the166 unchanged baseline files is outside this change, with their hashes preserved and original regressions retained. If those underlying assumptions fail, physical acceptance may fail or a baseline defect may remain; no source-level test result replaces hardware validation.
