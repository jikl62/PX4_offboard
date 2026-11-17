# ARM user-space DDS investigation

2026-11-16. Production controller/config/gates/TTL/service/builder untouched. Only tests/ros_arm64_ground_peer.py test harness edited, no commit. All earlier failures preserved. Result: current QEMU-user/PRoot environment cannot establish two-process Fast DDS discovery; a lower-level ARM multicast socket option reproducibly fails. The corrected test harness passes against real native host ROS+C++ and reaches RC_READY_WAIT_ARM with zero thrust and no commands, then exits gracefully.

## DDS/socket isolation

Environment: actual AArch64 Ubuntu22.04+ROS Humble binaries via QEMU6.2/PRoot5.4 uid1000, bundled installed px4_msgs/FC01. All isolated ROS tests use separate domain193, localhost1 except explicitly noted; no UART/Agent/hardware.

- Real ARM one-process rclpy publisher/subscriber VehicleStatus:358 sent/358 received, graph1 publisher/1 subscriber. Typesupport and serialized message path work within process.
- Real ARM two-process rclpy:193 sent/0 received, graph never discovers opposite endpoint (subscriber sees0 publishers and publisher sees0 subscribers).
- UDP-only Fast DDS profile excluding shared memory: same0 discovery/receive. Explicit SIMPLE discovery initial unicast peers127.0.0.1, localhost0,15seconds:361 sent/0 received. This does not isolate a production topic or safety gate problem.
- Direct ARM UDP unicast between Python processes:10/10 datagrams received.
- Direct ARM IP_MULTICAST_IF setsockopt for4-,8-,12-byte arguments fails errno92 ENOPROTOOPT. Identical host Python calls all succeed. QEMU_STRACE records syscall setsockopt(fd,IPPROTO_IP,IP_MULTICAST_IF,...)=errno92.
- Downloaded/extracted QEMU8.2.2 into user cache without changing system installation; same socket option failures and two-process ROS0 discovery. A separately compiled static ARM C reproducer fails directly under qemu-aarch64-static outside PRoot, further localizing limitation to QEMU-user/socket translation rather than PRoot or ROS policy. No production workaround applied.

## Harness corrections

1. Create output IPC directory0700 so original socket owner/permission checks pass without test shell umask dependence.
2. Continue the identical permanently-disarmed ground publication while waiting for graceful node exit; original harness stopped feedback then waited, creating its own source outage.
3. Keep one persistent nonblocking STATUS IPC client and consume streaming state without pausing publication. Only STATUS command sent; no flight actions. IPC buffer bounded1MiB, evidence samples periodically/at phase transitions.
4. Observe output with BEST_EFFORT QoS, compatible with native setpoint QoS, so zero/nonzero-target assertions actually receive messages.
5. Advertise one VehicleCommandAck publisher and one OffboardControlMode subscriber. Actual native graph contract requires both even when no flight task/commands are pending. ACK publisher sends no fabricated messages. Missing endpoints originally caused PUBLISHER_OR_TELEMETRY_CONFLICT when host DDS worked.

## Actual test outcomes

Earlier failed rootfake run `ground-smoke`: exit2 owned/private socket directory failure,25 batches0 callbacks. Preserved.
Earlier uid1000 run `ground-smoke-user`:742 batches0 callbacks, no fresh feedback, timed out after publication stopped, exit-9 by isolated harness cleanup. Preserved.
Corrected ARM environment `ground-smoke-user-fixed2`:1707 batches,0 callbacks,40 native snapshots, still no cross-process DDS; publication continued through30-second graceful wait. Correctly FAIL and exit-9 after isolated simulation timeout; no real service/hardware was killed. A separate intermediate fixed run reproduced output permissions before0700 fix and is preserved.

Corrected harness on host real ROS Humble+existing x86 C++ Stage3 node: PASS,603 feedback batches,4552 native callbacks,198 observed attitude targets all exact zero thrust,0 VehicleCommands; states IDLE→RC_PRESTREAM→RC_READY_WAIT_ARM; controller graceful exit0. This validates harness graph/QoS/stop behavior only, not ARM timing/physical flight. Raw /tmp/fc01-arm-ground-harness-host4/result.json and controller.log. Earlier host graph-deficient passes (targets0/IDLE) were not treated as preparation validation; final run proves READY explicitly.

`python3 -m py_compile tests/ros_arm64_ground_peer.py` and `git diff --check` PASS. No new mirror tests written; real DDS/node integration used as meaningful check.

## Evidence

All raw investigation scripts/profiles/socket/DDS logs and preserved controller runs copied into:
`/home/jikl62/.cache/fc01-arm64-20261116/dds-investigation-evidence/`
Summary: SUMMARY.json. Original failed directories remain unmodified. Logs dds-single-default.log, dds-dual-default.log, dds-dual-udp.log, dds-dual-explicit.log, dds-dual-qemu8.log, socket-probe.log, multicast-opt-probe.log, socket-opt-arm-direct.log.

Independent full ARM VM/systemd task remains separate and was informed about harness graph endpoints. Do not relax production gates/TTLs or claim the QEMU-user runs passed. Actual systemd+ARM DDS can be validated in the full guest where socket syscalls execute under its Linux kernel.
