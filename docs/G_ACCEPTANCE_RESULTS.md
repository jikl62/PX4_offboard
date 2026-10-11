# G1 acceptance results — 0.4.1-safety-hardening

Run evidence: G_TEST_OUTPUT.txt and G_BASELINE_TEST_OUTPUT.txt. PASS denotes offline policy/fixture evidence, never real vehicle permission. All numerical flight values used by tests are synthetic; delivered YAML remains unchanged with four gates false and flight values -1.0.

| Contract item | Result | Executed evidence / limit |
| --- | --- | --- |
| 1 stage-one nonzero final-boundary rejection | PASS | All five stage-one phases, each axis, ±0.4 and double denorm_min; both sink counts remain zero; stable STAGE_ONE_NONZERO_THRUST |
| 2 stage-one exact zero remains allowed | PASS | PRESTREAM/REQUEST_OFFBOARD/OFFBOARD_READY with four false gates; positive float zero in all three wire components; actual Runtime reaches READY |
| 3 flight_actions=false blocks Arm | PASS | Final Arm sink not called, FLIGHT_ACTIONS_DISABLED |
| 4 arm_enabled=false blocks Arm | PASS | Final Arm sink not called, ARM_DISABLED |
| 5 takeoff_enabled=false blocks Arm | PASS | Required by this contract; final sink not called, TAKEOFF_DISABLED |
| 6 parameters_confirmed=false blocks Arm | PASS | Final sink not called, FLIGHT_PARAMETERS_NOT_CONFIRMED |
| 7 Arm wrong phase | PASS | OFFBOARD_READY denied; only REQUEST_ARM admitted |
| 8 Arm shutdown | PASS | SHUTDOWN_LATCHED, zero sink calls |
| 9 Arm operator lease/session | PASS | Lease false and disconnected session separately denied |
| 10 Arm all valid inputs | PASS, policy only | Synthetic context/test sink; no ROS/hardware command |
| 11 nonzero any gate false | PASS | All 15 closed-gate masks rejected, target and heartbeat counts zero |
| 12 nonzero wrong phase | PASS | IDLE/SELF_CHECK/REQUEST_ARM/ARMED_GROUND/REQUEST_DISARM/SAFE_DISARMED/SAFETY denied; unknown enum also denied for zero |
| 13 nonzero shutdown/unhealthy | PASS | Shutdown, stall, stale attitude, invalid Z, failsafe, ownership conflict, lost lease, unarmed, invalid/lateral/positive thrust cases |
| 14 all four status gate fields | PASS, source/fragment | Actual flight_gates_json fragment checked; adapter compiles against fixture; actual ROS status transport UNKNOWN |
| 15 attitude profile owns no Trajectory publisher | PASS | Original publisher_plan/ownership tests plus source audit; planner byte-identical |
| 16 supervised heartbeat | PASS, offline | Both sinks unreachable on policy denial; Runtime prepare/disconnect and original fault tests; actual DDS timing UNKNOWN |
| 17 command 400 sequential ACK/watermark | PASS | Original test_ack_lifecycle/test_attitude_runtime unchanged; ack()/track() byte-identical across three releases |
| 18 regular file at socket path | PASS | C++ constructor and Python update fixtures refuse, inode/current retained |
| 19 symlink/unexpected path type | PASS | Update dangling symlink/FIFO retained; lock symlink refused |
| 20 confirmed live listener refuses update | UNKNOWN | Test exists but native socket() forbidden (EPERM), explicitly SKIP; existing-path refusal policy tested separately |
| 21 stale-looking socket | PASS, filesystem policy | Actual mknod S_IFSOCK inode retained by constructor/helper; LIVENESS_UNKNOWN diagnostic; no connect/delete |
| 22 ambiguity/race/unknown fail closed | PASS, cooperating fixtures | Busy flock, changed lock pathname, socket path appearing during activation, unknown service state all refuse; arbitrary legacy/noncooperating race safety UNKNOWN |
| 23 install never auto-switches current | PASS, fixture | Original install script byte-identical; fake colcon install with existing/absent fixture current; explicit update and rollback tested |

Packaging results are in the external PACKAGE_VERIFICATION.txt: clean ZIP/TAR source, patch replay onto pristine 0.4.0, all file bytes and executable bits match, SHA256 manifest validates. G_INVARIANTS.json records the protected hashes. No real install/update/service operation was executed.

0.4.0 regression: 5 C++ program exits 0 (native socket subcheck SKIP); Python 8 tests, 7 PASS / 1 SKIP. 0.4.1: 8 C++ program exits 0 (native socket subcheck SKIP); Python 14 tests, 12 PASS / 2 SKIP. Adapter fixture syntax, 12 shell syntax checks and 9 Python AST checks PASS. These are not Humble build/link, DDS or FC01 evidence.
