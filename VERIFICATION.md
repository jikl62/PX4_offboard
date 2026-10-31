# Verification record

## R2 startup-gate correction

The first device run exited before socket creation. The wrapper incorrectly enabled the legacy `flight_enabled` flag with the dedicated attitude gates. Unchanged native `runtime.hpp` Config::validate() explicitly rejects that combination with `attitude_altitude uses separate flight_actions_enabled gates`. The previous Python test encoded the same wrong expectation; the original C++ regressions did not consume the wrapper-generated YAML. Their passing results did not cover this integration boundary.

Added a contract probe that derives parameter/member mappings from the actual native ROS adapter and compiles/calls the unchanged native Config::validate() using generated session parameters. The flight-session test failed with that exact error before the fix; check mode and an intentional legacy-flag rejection passed. Corrected only the flight-session flag selection: legacy `flight_enabled=false`, five dedicated action/confirmation flags true. Original native controller, project YAML and numerical parameters are unchanged. Final R2 verification results follow below.

R2 full suite: 43 tests passed in 12.444 s, including three actual native-config contract tests. The replacement bundle uses a separate R2 extraction directory so existing unequal R1 files are preserved. The one-time repair command for installed R1 changes only the same gate range and keeps a backup; subsequent starts use the installed start.sh rather than the old uncorrected desktop installer.

Scope: host validation of an operator entry for an existing native ARM64 controller. No actual Orange Pi, PX4 actuator or powered-flight access was available. Host tests do not establish hover stability, airframe calibration or navigation readiness.

Before implementation, behavioral tests failed for the missing configuration/Mission, launcher and installer implementations. The EOF regression failed with lost final state and then passed after correction.

## Executed checks

- Task 1: 15 configuration/Mission tests passed.
- Task 2: 28 tests passed, including real Unix stream socketpair and child-process integration for closed-gate check mode and explicit takeoff → quit → native descent while preserving heartbeats.
- Four unchanged C++ reference regressions compiled and passed with `g++ -std=c++17 -Wall -Wextra -Werror -pedantic`: `test_authority_runtime`, `test_authority_boundary`, `test_r632_skewed_176_ack`, `test_r632_prompt_176_ack`.
- Complete final Python suite: 40 tests passed in 11.814 s, including 8 offline-installer integrity/preservation/input-stream tests. Installer tests first failed for the missing implementation. Pending-Arm quit/EOF/signal cases are three additional subcases within one integration test.
- Python source parsed with Python 3.10 grammar. Bash syntax checks passed for start.sh and the generated self-extracting script. Bundle build passed.
- One independent whole-change read-only review completed. It independently passed the original 36 tests and found two Important lifecycle defects: terminal-output errors could escape recovery, and synchronous ZIP compression could interrupt heartbeats. Both were fixed. New broken-output, blocked-output and >3 s slow-report regressions were observed failing before the changes, then passed. Display now uses a bounded daemon queue; only a small checkpoint is saved while the native node is alive; ZIP generation waits until node exit.

Review scope decisions: actual calibration, hover stability and powered-flight/RC recovery remain hardware work; ARM64/ROS2 execution and pathname socket binding remain launch-time device checks; publisher signatures are outside this integrity-only installer. These are accepted scope boundaries, not waived test failures. No hardware-flight completion is claimed.

Final source checks: Python 3.10 grammar parsing, Bash syntax, `git diff --check`, self-extractor build and install-only extraction. The generated artifact is rechecked after documentation is finalized.

Named AF_UNIX socket creation is restricted on the development host. Wire/lifecycle integration therefore uses a real AF_UNIX socketpair inherited by a child process. The actual controller's pathname binding and ARM64/ROS2 execution remain device-side checks performed at launch, with automatic evidence capture.

The wrapper preserves original numerical parameters, C++ authority/clock/ACK/publication/tilt guards and the original project files. It only creates a per-session configuration. `--flight` opens that session's action/confirmation flags as an explicit operator selection of the existing candidate parameters; actual takeoff still requires the live operator command and native admission. No automatic retry of TAKEOFF, Arm-only or unsupported horizontal commands exists.
