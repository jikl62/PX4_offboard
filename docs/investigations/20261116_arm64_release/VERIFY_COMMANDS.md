# Reproduction commands and evidence boundaries

Run from the FC01 repository on the x86 development host:

```bash
PYTHONPATH=tests:. python3 -m unittest \
  test_navigation_service test_navigation_prebuilt_service \
  test_navigation_prebuilt_install test_arm64_bundle -v
bash -n prebuilt_boot.sh dist/arm64/install_fc01.sh
git diff adc55d0b7c67e80bf9d77b1082208c898f95b756 -- \
  native/src native/config native/tools launcher.py standby.py navigation_service.py
(cd dist/arm64 && sha256sum -c FC01_OrangePi_20261116_stage3_arm64.tar.gz.sha256 && \
 sha256sum -c install_fc01.sh.sha256)
```

Recorded results: 69 host tests passed, shell checks passed, control/config paths unchanged. The corrected ground harness was separately exercised through actual host Humble DDS; raw results are in `evidence/host-ground-harness.tar.gz`.

Actual ARM compilation used Ubuntu 22.04 ARM64/GCC11/ROS Humble under QEMU user emulation with the pinned SDK px4_msgs sources. `evidence/ARM64_BUILD.json` binds all native inputs and installed outputs. The exact build command (including native source paths, sequential executor, fixed Python and BUILD_TESTING=OFF) is recorded in `evidence/ARM64_BUILD.json` under `command`; the full output is retained in `evidence/arm-colcon-build.log`. `build_arm64_bundle.py` packaged that install tree, validated input/output bindings and independently re-extracted the result.

Actual installation integration used a complete QEMU-system-aarch64 Ubuntu 22.04 ARM64 guest, real systemd PID1, the production installer Backend, Supervisor and ARM controller. See `evidence/vm/COMMANDS_zh.md` for its real commands. The 43 ARM unit tests are backend-injection transaction tests; real service installation/repeat/rollback are separate recorded integration scenarios.

The VM supplied synthetic, permanently disarmed grounded PX4 DDS feedback and a PTY-holding Agent substitute. It did not exercise real XRCE, UART, PX4, MTF-02P or Orange Pi hardware. QEMU-user cross-process DDS failures are retained rather than reported as passes. The real automatic-rollback integration failure used the previous archive with a 30-second install startup deadline; the final archive's 180-second acceptance succeeded. Control feedback deadlines were unchanged.
