# Final ARM64 delivery artifact independent review

Result: PASS for archive/installer identity and syntax, payload/source/build binding, AArch64 architecture and relocated dependency selection. Read-only review; no deployment, service, controller, Agent or UART operation. No blocking findings.

Exact reviewed files:

- `dist/arm64/FC01_OrangePi_20261116_stage3_arm64.tar.gz`
- Archive bytes: 6,497,136
- Archive SHA256: `3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123`
- `dist/arm64/install_fc01.sh`
- Installer SHA256: `7b35de4b7e498d46c56ac7b9abbf032c2ba64d1a76ab11177f9391bc4fc4e956`

The actual archive digest/length and installer digest match `DELIVERY.json`; the archive digest also matches the adjacent `.sha256` file. Version is `20261116_stage3`, source commit `56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a`, control candidate `adc55d0b7c67e80bf9d77b1082208c898f95b756`, Ubuntu 22.04, ROS Humble, AArch64.

Production `navigation_prebuilt_install.extract_checked()` accepted a new temporary extraction: 4,973 payload files plus manifest, 4,974 total. This exercises pinned compressed SHA, bounded safe extraction, full exact file coverage/hashes, no special/link/deployment members, controller executable mode and AArch64 ELF validation. `build_arm64_bundle.validate_evidence()` also passed on the extracted build record, whose complete JSON equals the original cached `ARM64_BUILD.json`.

Independent byte comparisons:

- All 98 non-install source files exactly match committed Git archive `56424d7`.
- All 53 selected native source/tool files exactly match control candidate `adc55d0`.
- All 4,874 installed files, including every ELF, exactly match `/home/jikl62/.cache/fc01-arm64-20261116/build-work/install` after ordinary file dereference.
- Exactly 26 ELF files; all are 64-bit little-endian AArch64 (machine 183). Native controller SHA256 is `c446afae3d2a262bff132232cac77a40d2c9499ce8ea6a765d6d52c97f02cdc7`; configuration checker SHA256 is `8c0db7f0edcffab545de8ab19cd63e9dfecd9d3620eee044ee87008c69e3fe15`.

Relocation/dependency selection:

A second strict temporary extraction under a fresh unpredictable directory was checked in the existing Ubuntu 22.04 ARM64 user environment, using the production entry sequence: clear old overlay variables, source `/opt/ros/humble/setup.bash`, source only the relocated bundle's `install/local_setup.bash`. No ROS context, controller or publisher was created. The nine bundled message classes used by the service passed `rclpy.type_support.check_for_type_support`. `ldd` resolved every one of the 26 ELF files with no missing libraries. Every selected library path belonged to the relocated bundle's `install`, `/opt/ros/humble`, or Ubuntu `/lib`/`/usr/lib`. AMENT/CMake/Python/library environment paths contained only the relocated bundle plus Humble, with no old SDK/build prefix. Imported project type-support memory mappings selected the relocated bundle.

Raw relocated dependency/import/environment results: `/tmp/fc01-final-artifact-ldd-review.json`.

Qualification: two duplicate `install/local/lib/python3.10/dist-packages/{jikl_fc01_system,px4_msgs}/lib*__rosidl_generator_py.so` copies retain `/delivery/build-work/build/<package>` in RUNPATH text. The active controller/checker/project support libraries use `$ORIGIN:$ORIGIN/..`; the actual imported generator libraries and all `ldd` selected dependencies came from bundled `install/lib`, Humble and Ubuntu. Thus the supported runtime entry does not select an old SDK/build prefix, but the stronger claim that no build path string remains anywhere in the archive would be inaccurate. This does not affect the tested relocation sequence.

Standalone installer syntax/source pin:

Independent parallel review `/tmp/fc01-final-installer-artifact-review.md` confirms `bash -n` PASS and extracted heredoc Python `compile(..., 'exec')` PASS without execution. Embedded Python exactly matches committed `56424d7:navigation_prebuilt_install.py` after the one expected replacement that pins `--expected-sha256` default and its single allowed choice to this archive digest; no other source change. Both actual delivery hashes and installer pin values agree. All 27 non-install Python files inside the archive also compile without execution. Installer dependency checks clear inherited overlays and select only system Humble plus candidate `install/local_setup.bash`.

Actual real-systemd VM install/repeat/rollback evidence is a separate build-probe task; these artifact checks do not establish hardware flight qualification.
