# Orange Pi FC01 control entry

Goal: reuse the installed ARM64 R6.3.2 attitude/altitude controller, verified DDS serial link at 115200, and existing px4_msgs; replace repeated terminal diagnostics with one self-contained launch file and a live operator console. First establish takeoff/height hold/landing. Vision navigation is the next stage, not silently enabled here.

The user explicitly asked to execute without repeated questions. Implementation choices below are ours, not claims of additional hardware validation.

Architecture: Bash loads Humble and the existing install_r632 overlay. A Python supervisor uses the existing C++ executable, its private Unix control protocol, and per-session YAML copies. A Mission state machine sequences AUTHORITY_RC, ATTITUDE_READY, and TAKEOFF only after an explicit interactive takeoff command. The C++ controller remains the sole flight command publisher and all its authority, source-clock, ACK, takeover, estimator-reset, tilt, height, and publication checks remain authoritative.

Modes: default --check is closed-gate and non-actuating. Explicit --flight enables the existing action/parameter-review/RC-route flags only in the session copy and opens an interactive console; it never arms on launch. The operator's --flight selection means using the displayed existing candidate parameters and previously selected RC-takeover route; it is not a claim of flight calibration. Actual Arm is requested only through the original C++ TAKEOFF path after the user enters takeoff [height]. One takeoff submission per controller process, no automatic retry. LAND cancels queued preparation. No autonomous-no-RC path, no ARM_ONLY, no unsupported HOLD/VEL commands.

Existing installation: ~/FC01_ARM64_PORT_20261020/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE; config/attitude_altitude.yaml; install_r632/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node. Agent: ~/fc01_agent_build_offline/MicroXRCEAgent serial -D /dev/ttyS2 -b 115200. Reuse a single matching existing Agent, refuse unknown/incorrect owners, never stop reused processes. Do not edit source/install/config or PX4 parameters.

Live observations supplied by user: saved SER_TEL3_BAUD=115200; 480 new Ping replies for 480 attempts, no added expiration or reconnection; receive rates 2/20/20/1/1 Hz; five key streams have increasing stamps, attitude/local maximum arrival gaps 96.2/86.1ms; land max 1.0356s; z/vz valid; landed and at_rest true. Source and installed header bind aa_land_sample_max_gap; YAML is 1.25s, land_timeout 2.5s. These are bounded bench observations, not flight results.

Ownership and shutdown: new private short socket path each run, no stale socket deletion, no killing unrelated controller. Keep console heartbeat and Agent alive while the owned controller handles LAND/shutdown. Never forcibly kill a potentially airborne controller. Stop an owned Agent only after the controller has exited and fresh disarmed/landed evidence exists; reused Agent is never stopped. Log and report unknown state rather than inventing a safe result.

Deliverable: a single offline self-extracting .sh, reusable entry files, README, verification report and tests. A report ZIP is generated automatically on refusal/exit. No network installation and no automatic boot service. Tests cover guard sequencing, stale/out-of-order responses, single-attempt behavior, numeric configuration preservation, Agent ownership, private socket paths, lifecycle order and installer integrity.

Approaches: reuse installed native controller (selected to minimize changes and deployment time); rebuild/copy the entire project (unnecessary without a changed control algorithm); write a new Python flight controller (rejected because it would discard the reviewed C++ boundaries).

Vision interface: retain future EKF external-odometry/native PX4 velocity-position path; velocities plus angular rate alone do not establish a position hold. No fabricated vision data or validity flags.
