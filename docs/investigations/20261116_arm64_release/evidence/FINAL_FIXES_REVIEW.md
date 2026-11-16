# Final prebuilt installer fixes: independent read-only review

Result: PASS. No blocking or nonblocking findings in the reviewed changes.

Reviewed committed changes:

- `416e7e9`: runtime dependency checking no longer serializes the complete release manifest into one process argument. The actual generated shell/Python command only receives the release directory; the 5000-member regression checks the real Backend command and nested Python argv access.
- `5ff1255`: `WorkingDirectory` uses scalar-path serialization, while `ExecStart` retains argument quoting. The helper rejects control characters, backslashes, relative paths and boundary whitespace, and escapes systemd percent specifiers. A real inert `systemd-analyze verify` test exercises spaces and percent signs without starting a service.
- `56424d7`: `Backend.live_state()` performs the potentially slow systemd/process/boot identity reads before reading the newest atomic Supervisor snapshot. Supervisor/controller/boot/process-start bindings remain enforced. The native monotonic snapshot limit remains 0.25 seconds and accumulated feedback source TTL validation is unchanged. The regression demonstrates acceptance of a newer snapshot written during a 0.13-second service query and continued rejection of an unchanged stale snapshot.
- `56424d7`: only the new-version `verify_active()` default acceptance budget increases from 30 to 180 seconds, through `STARTUP_ACCEPTANCE_SECONDS`. Existing stop and original-version recovery deadlines remain 30 seconds. Acceptance still requires the existing fresh, grounded, disarmed, zero-thrust evidence, candidate executable/hash/cwd, inherited full runtime policy and DDS environment. The deterministic regression accepts a controller appearing after 40 seconds and rejects a permanently absent controller at the configured deadline.

Validation rerun against current checkout:

```
PYTHONPATH=tests:. python3 -m unittest test_arm64_bundle test_navigation_prebuilt_install test_navigation_prebuilt_service -q
Ran 43 tests in 0.500s: OK
python3 -m py_compile navigation_prebuilt_install.py tests/test_navigation_prebuilt_install.py tests/ros_arm64_ground_peer.py
git diff --check
```

All checks passed. Review did not operate any host service, UART, Agent or controller.

VM motivation and scope:

- `actual-install-6.log` records new-version acceptance failure and successful automatic restoration of the original configuration and running state.
- `install6-automatic-rollback.log` records transaction phase `failed_restored` with no candidate controller appearing before the original acceptance deadline.
- `standalone-prebuilt-check.log` records successful standalone payload/dependency/configuration checking at `real 0m46.377s`, longer than the prior 30-second acceptance budget.
- Independent evidence review `/tmp/fc01-prebuilt-timing-evidence-review.md` corroborates candidate service start/stop about 31 seconds apart, restored old actual executable and original runtime/unit hashes, current READY_WAIT_ARM ground feedback, target 1.23 m, RC slot 1 and shared Agent PID 671. The restored unit preserves configured DDS settings and fresh feedback confirms working DDS; these logs do not separately dump the entire restored process environment.
- This supports an installer startup wait adjustment. It does not establish a flight timing change or physical-hardware qualification. A real ARM64 VM rerun of the new deadline is owned by the build-probe agent and remains separate evidence.

Ground harness handoff:

`tests/ros_arm64_ground_peer.py` changes are finished and ready for root to commit. The private output directory, persistent nonblocking STATUS observation, BEST_EFFORT subscriptions, empty advertised ACK publisher, heartbeat subscription and continued permanently disarmed feedback during graceful exit are present. No further QEMU-user debugging or harness edits are pending.

Existing host ROS/native result `/tmp/fc01-arm-ground-harness-host4/result.json` was re-read: PASS, 603 feedback batches, 4552 native callbacks, 198 observed zero-thrust attitude targets, zero VehicleCommands, phases IDLE -> RC_PRESTREAM -> RC_READY_WAIT_ARM, and graceful native exit 0. This is host software integration evidence. ARM QEMU-user multi-process DDS remains an explicitly documented environment failure and is not claimed as passing.
