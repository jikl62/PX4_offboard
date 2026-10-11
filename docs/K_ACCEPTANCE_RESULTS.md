# K acceptance results — 0.4.3-rc-interface

All entries below are offline unless explicitly marked FIELD-EVIDENCE. No FC01, Agent, Arm, Takeoff, parameter write, or real nonzero setpoint was used.

| # | Requirement | Result | Primary regression |
|---|---|---|---|
| 1 | default authority UNSET | PASS | `test_authority_runtime.cpp`, `test_authority_tools.py` |
| 2 | no ManualControlSetpoint => rc_link false | PASS | `test_rc_interface.cpp` |
| 3 | invalid ManualControlSetpoint => false | PASS | `test_rc_interface.cpp`, `test_authority_telemetry.cpp` |
| 4 | non-RC data_source => false | PASS | same |
| 5 | first RC raw sample not sufficient | PASS | `test_authority_review.cpp`, `test_rc_interface.cpp` |
| 6 | second fresh advancing raw sample can ready link | PASS | same |
| 7 | duplicate raw timestamp cannot refresh activity | PASS | `test_authority_telemetry.cpp`, `test_rc_interface.cpp` |
| 8 | stale => false | PASS | `test_rc_interface.cpp` |
| 9 | first sample after stale gap not sufficient | PASS | `test_authority_review.cpp`, `test_rc_interface.cpp` |
| 10 | second fresh sample after stale gap can recover | PASS | `test_rc_interface.cpp` |
| 11 | missing ManualControlSwitches keeps rc_takeover unready | PASS | `test_rc_interface.cpp` |
| 12 | switch stale/unknown/Kill ON fail closed | PASS | `test_authority_telemetry.cpp`, `test_rc_interface.cpp` |
| 13 | Kill retention preserved | PASS | `test_authority_telemetry.cpp`, existing authority regressions |
| 14 | switch/mode takeover one-way latch preserved | PASS | `test_authority_review.cpp`, `test_authority_runtime.cpp` |
| 15 | nav_state departure stops/releases and does not reacquire | PASS | existing authority harness/runtime scenarios |
| 16 | sticks_moving diagnostic-only; cannot bypass route gates | PASS | `test_rc_interface.cpp` |
| 17 | AUTONOMOUS_NO_RC does not depend on RC telemetry | PASS | `test_rc_interface.cpp`, authority harness |
| 18 | four flight gates remain independent final enforcement | PASS | `test_authority_boundary.cpp`, `test_publication_boundary.cpp` |
| 19 | stage-one exact-zero preserved | PASS | publication boundary/runtime regressions |
| 20 | command 176/400 ACK lifecycle preserved | PASS | `test_ack_lifecycle.cpp` |
| 21 | socket/update/install regressions preserved | PASS | C++ socket tests + Python socket/tools tests |
| 22 | JSON diagnostics distinguish source/switch/Kill/latch/autonomous state | PASS | `test_authority_tools.py`, `test_rc_interface.cpp` |

`bash scripts/test.sh` on pristine 0.4.2: PASS. `bash scripts/test.sh` on 0.4.3: PASS. The real ROS adapter also passes the existing narrow compile fixture updated to the exact pinned `ManualControlSetpoint` API shape; this fixture is not a ROS2 Humble build/link or DDS proof.

Environment for K execution: Debian 13, g++ 14.2.0, Python 3.13.5; `ros2` and `colcon` absent. Therefore real Ubuntu 22.04 / ROS2 Humble build/link with the exact generated `px4_msgs` package is UNKNOWN.
