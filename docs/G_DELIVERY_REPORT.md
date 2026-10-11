# G DELIVERY REPORT — 0.4.1-safety-hardening

日期：2026-10-04。角色：智能体 G。交付性质：从冻结 0.4.0 单独复制的离线安全加固源码候选。G1 已实现最终 Arm 四 gate、第一阶段精确零推力及非零推力阶段检查、可观察故障锁存、诊断后拒绝的 socket 更新。I-01 在源码和离线 policy/sink 测试范围内闭合；这不是新版本真机验收或飞行批准。

## 1. Files actually read

先完整解压上传的工程包，按 00→01→02→03→04→05→06 顺序阅读正式合同；复制原文至 docs/g_contracts/。25 项交接 SHA256 全部通过。G1 完成前没有执行 07/G2。

直接审查实际 0.3.1 baseline/ 和 0.4.0 candidate/：Runtime、ground_link、protocol、UnixServer、velocity_core、实际 ROS node、ACK lifecycle 测试；候选新增 attitude_altitude_core、attitude_reference、vertical_controller、attitude_ros_parameters、attitude_wire、setpoint_ownership，以及 attitude runtime/vertical/wire/socket/tools 测试；install/update/common/start/run/agent/build_agent、deploy/fcctl/start/pc、VERSION、配置、CMakeLists/package.xml。逐文件字节清单见 G_INVARIANTS.json；新实现和新增测试也实际复核。历史 .before_message_version_fix 源码保留原样，不由 CMake 编译。

直接阅读 D_RELEASE_SAFETY_REVIEW_FC01_0.4_20261004.md、F_DELIVERY_REPORT_0.4.0-attitude-altitude.md 及新证据 REAL_UBUNTU_BUILD_AND_ACTIVATION_EVIDENCE.txt；A_ACK_ENGINEER_REPORT、B_PX4_PROTOCOL_AUDIT、C_CONTROL_ARCHITECTURE_REPORT、E_ORANGE_PI_PORTABILITY_AUDIT 的 ACK、协议、架构、移植结论/相关约束作为支持证据审查。没有把旧报告中的 PASS 自动转移给新版本。

只使用 06 列出的 PX4 v1.16 官方接口资料核对态度入口、q_d/thrust_body、OffboardControlMode、ACK 字段；没有使用 main 替代。链接：

- https://docs.px4.io/v1.16/en/flight_modes/offboard
- https://docs.px4.io/v1.16/en/ros2/user_guide
- https://docs.px4.io/v1.16/en/msg_docs/VehicleAttitude
- https://docs.px4.io/v1.16/en/msg_docs/VehicleAttitudeSetpoint
- https://docs.px4.io/v1.16/en/msg_docs/OffboardControlMode
- https://docs.px4.io/v1.16/en/msg_docs/VehicleCommandAck

精确 px4_msgs commit 的在线原始文件访问未成功；本次未获得真实该 commit 头文件/build 证据，标为 UNKNOWN。pin 原样保留为 392e831c1f659429ca83902e66820d7094591410。两份硬件 PDF 随包保留/校验，本任务未进行设备调试或依赖其未核实内容。

## 2. D I-01 root cause / boundary analysis

0.4.0 的态度 Arm 调用处检查 flight_actions_enabled/arm_enabled/takeoff_enabled、RequestArm、连接和 shutdown，缺少第四 gate 的独立终端检查。通用 publish_command() 直接发送，其他/未来调用者可绕过该局部检查。配置验证及上游状态机不能替代最终 sink 的独立授权。

0.4.0 的 wire() 仅验证 publish、四元数和有限 [-1,1] 推力，无 profile/phase/gates/shutdown/lease/health 上下文，因此上游错误的有限非零 stage-one 输出仍能进入 ROS target 和 heartbeat。status 仅显示前三 gate，缺少 flight_parameters_confirmed；这是本次一并处理的可观察性缺口。

D I-02 的旧构建缺证据由用户提供的 Ubuntu22.04/Humble 两包成功 build 解决，但只适用于冻结 0.4.0。用户真实 current 已显式在 0.4.0；本次从未接触它。

## 3. Exact hardening design

publication_guard.hpp 提供无 ROS/DDS、无副作用的 BoundaryContext→Decision，稳定原因 enum/string。Runtime 只新增只读 lease/reference/context 查询，不改 ACK 或总体状态机。ROS adapter 在最终发布前重新采样单调时间、telemetry freshness/ownership、phase、配置和 shutdown/session/lease。

Arm：profile=attitude_altitude、四 gate 全真、显式飞行参数 validate、REQUEST_ARM、shutdown=false、control fresh、已连接且 live interactive lease、当前 disarmed、reference/flight_started、真实 offboard、landed/at_rest/静止时间/Z-VZ、电池/RC/preflight，以及所有公共健康检查。失败时无 Arm sink 调用，PUBLISHER_BOUNDARY_<reason> 输出并锁存。

态度：先检查上游 double 值，stage-one 任意有限非零（含侧向分量、正推力和 denorm_min）拒绝，不靠 float 下溢隐藏错误，也不静默 clamp。正常第一阶段三种发布阶段仍允许 gates=false、level roll/pitch+local yaw、精确正零 [0,0,0]。IDLE/SELF_CHECK 正常不发布；SAFE_DISARMED/SAFETY/未知 enum 均不能发布。

第二阶段非零只在 THRUST_RAMP/ASCEND/HEIGHT_HOLD/DESCENT/LANDED，要求四 gate、有效配置、reference、armed/offboard、fresh/valid telemetry、ownership、未超高度/倾角、battery/RC 和合法 multicopter thrust；ASCEND/HOLD/RAMP 还要求 live session/lease。DESCENT/LANDED 保留健康受控下降的断线语义；关闭 gate 或失去健康不能继续输出。

publication_boundary.hpp 先许可，再 wire 编码，再 target sink→mode sink。失败时两 sink 均不可达；node 清空整个 output、清空 last output、fault core、锁存故障，不允许另一请求解除。没有单独的 attitude heartbeat 定时器。

有意收紧：任何 flight/sim Arm 最终也被拒绝，因为本合同要求所有 Arm 都是 attitude_altitude；shutdown 后所有 attitude target/heartbeat 停止，不能继续旧的非零 shutdown descent，交还 PX4/operator。非 Arm ground/Offboard/Disarm 命令字段与原 ACK 机制未重写。双 DDS 消息不是原子事务，异步信号和调度仍有物理时间窗口，不能声称硬实时原子保证。

status 保留原有字段，增加第四 gate、heartbeat_enabled、attitude_output_enabled、publication_block_reason、publication_fault_latched。thrust_body 在停止/拒绝时显示清空后的零值；enabled 表示本 tick 的发送许可/调用结果，不是飞控接收证明。

## 4. Dangerous publisher boundary table

| 最终 sink | 调用位置与 guard | 独立约束 | 拒绝结果 |
| --- | --- | --- | --- |
| VehicleCommand 400/param1=1 | publish_command → publish_arm_at_boundary | profile、四 gate、有效参数、REQUEST_ARM、shutdown、connected/live lease、disarmed、健康/ground/reference/preflight | 不发送 command；原因锁存，清空 output |
| VehicleAttitudeSetpoint（stage one） | tick → publish_attitude_at_boundary | publishable phase、health/reference、连接/preparation lease、disarmed；原始 thrust 必须全 0 | 上游非零明确 STAGE_ONE_NONZERO_THRUST；不发 target/mode |
| VehicleAttitudeSetpoint（nonzero） | 同一 paired boundary | 四 gate、显式 flight phase、有效参数、armed/offboard、健康/envelope/推力符号/范围、适用的 lease | 不发 target/mode；fault latch |
| OffboardControlMode attitude heartbeat | paired target 成功调用后，唯一 mode sink | 同一 policy 和 wire packet；无独立许可 | target/policy/encoding 失败不发 heartbeat；sink 异常退出 |

wire() 本身仍是纯字段适配器，不是授权 API；旧 wire 文件字节不变。trajectory/velocity 历史 profile 路径保持所有权隔离，不能把它误称态度模式的发布路径。

## 5. Stale-socket design and race analysis

采用诊断后拒绝，不自动删 stale-looking socket。ECONNREFUSED、无进程、service inactive、ss 无 listener 单项均不能证明未来不会启动。更新不再连接 controller 的 update-check/stop，不停止或启动服务，避免诊断连接触发原会话 disconnect 控制语义。

UnixServer 在启动前持有 socket_path+'.lock' 的整个生命周期 advisory flock，验证 owned/non-group-world-writable directory、O_NOFOLLOW regular lock、uid/dev/inode/权限。既存路径：非 owned Unix socket（普通文件、symlink、FIFO 等）拒绝；owned Unix socket 打印 SOCKET_PRESENT_LIVENESS_UNKNOWN 并保留。正常退出只移除本实例新建且仍匹配 dev/inode/type/uid、同锁身份的 socket；永久不删除 .lock。

Python update 与服务器共享 flock；built release 和 releases 范围验证后，service 状态必须明确不 active（systemctl is-active 的约定 exit 3；失败/未知/超时拒绝）。持锁期间要求 socket 路径不存在，使用自己 UUID 的 temporary current symlink；切换前再次检查 service、锁 pathname identity 和 socket，再 os.replace 原子换 current。只清理自己的临时 symlink，不碰任何既存 socket/lock。无自动恢复任务。

tools/socket_safety.py 只 lstat/打印诊断，不 connect、不杀进程；提示独立确认所有 starter/service 停止且不会重启、无 listener、重新确认 owned socket 类型和 inode，然后人工执行 shell-quoted 的 rm -- <path>。打印命令从未由程序执行；不删除 lock 文件。

锁可消除遵守同一协议的 0.4.1 starter/update 之间的竞态，重复检查可拒绝观察到的路径变化。它不能约束旧 0.4.0 或任意不遵守锁的进程，不能证明同 UID 恶意 pathname 替换不可发生。升级旧版本必须人工 quiesce 所有启动途径；任意外部进程在最后一次检查之后启动的安全性为 UNKNOWN，不作无条件 race-proof 声明。socket 已存在时始终中止，属于本合同允许的 fail-closed 方案。

## 6. File-by-file diff summary

| 文件（相对 release root） | 改动 |
| --- | --- |
| VERSION、src/jikl_fc01_system/package.xml | 新 release 0.4.1-safety-hardening / package 0.4.1 |
| include/.../publication_guard.hpp（实际在 src/jikl_fc01_system 下） | 纯四 gate/phase/health/lease/shutdown/zero policy、稳定原因、四 gate JSON |
| include/.../publication_boundary.hpp | 最终 Arm sink / paired attitude-mode sink 函数 |
| include/.../attitude_altitude_core.hpp | 仅新增只读 operator/preparation lease 查询 |
| include/.../runtime.hpp | 仅新增 guard include 和新鲜 publication_context；ack()/track() 不变 |
| src/jikl_fc01_system/src/fc01_system_node.cpp | 最终统一 Arm guard、受监督 pair、fresh context、fault latch、加法 status 字段 |
| include/.../socket_lease.hpp、include/.../unix_server.hpp | lifetime flock、拒绝所有已有路径、own-inode 正常清理、无 stale auto-delete |
| tools/socket_safety.py、tools/update.py、scripts/update.sh | 诊断后拒绝、cooperating activation、recheck/atomic symlink，不连/删/stop/start |
| scripts/test.sh | 增加实际 adapter 的 fixture syntax 编译；原 C++/Python测试继续运行 |
| tests/test_publication_boundary.cpp | 注入非法上游输出、四 gate/phase/lease/shutdown/health 和 sink 计数 |
| tests/test_publication_runtime.cpp | 实际 Runtime 在 gates=false 走到 READY→boundary、wrong ACK/true ACK、断线停止 |
| tests/test_socket_boundary.cpp、tests/test_socket_safety.py | owned S_IFSOCK 保留、flock、异常路径/竞态/service/update fixtures |
| tests/adapter_compile_check.py | 窄 rclcpp/px4_msgs API/type fixture 编译真实 node；明确不是实际 ROS build |
| tests/test_socket.cpp | 仅临时 fixture 清理适应永久 lock 文件；不改生产安全断言 |
| tests/test_tools.py | 仅版本字符串预期从 0.4.0 改 0.4.1；其余原 install/rollback 等回归保留 |
| README_zh.md、docs/G_*、docs/g_contracts/*、docs/superpowers/plans/*safety-hardening.md | 当前候选说明、正式合同原文、测试/不变量/审查/RED-GREEN/交付证据 |

完整 unified diff 是外部 fc01_system_v0.4.1-safety-hardening_vs_0.4.0.patch。原 0.4.0 源码及其历史 F 文档未被原地修改；历史文件属于 0.4.0 证据，不代表新候选验收。

## 7. New/old tests and raw results

原始 0.4.0：实际执行未修改候选 scripts/test.sh，5 个 C++ program exit 0（其中 native AF_UNIX subcheck SKIP），Python Ran 8 / OK(skipped=1)，7 个实际通过。完整输出 G_BASELINE_TEST_OUTPUT.txt。

新 0.4.1：8 个 C++ program exit 0（ACK、attitude runtime、vertical、wire、publication boundary、publication runtime、socket、socket boundary），Python Ran 14 / OK(skipped=2)，12 个实际通过。native AF_UNIX listener / ready-client 被禁止并明确 UNKNOWN。真实 inode mknod 和 flock/文件系统路径测试实际运行。完整输出 G_TEST_OUTPUT.txt；23 项映射在 G_ACCEPTANCE_RESULTS.md。

g++13.3 C++17 -Wall -Wextra -Werror -pedantic；实际 node 用窄类型/API fixture 做 -fsyntax-only，通过。12 个 shell 的 bash -n 和9个 Python AST 通过。无真实 ROS/Humble/colcon，该 fixture 不证明 exact pinned header、链接或 DDS。

RED/GREEN 原始输出 G_RED_GREEN.txt：缺最终 guard 的 RED assert exit134；旧 updater 对 file/symlink/FIFO 仍切 fixture current 的3个错误行为被 RED 捕获；synthetic unknown Phase 零输出原先放行被 RED 捕获后修成 exhaustive allowlist。初次 live listener 测试在环境 socket() EPERM 处 ERROR，是测试环境/harness 处理缺口，修为显式 SKIP，不能算产品 RED 或已验证 live listener。最新完整 suite 全部可执行检查通过，skip 不计 PASS。

whole-change 作者审查记录 G_CODE_REVIEW.md。新鲜独立审查 agent 因额度限制未运行、未提供任何结论，因此 independent review=UNKNOWN；不伪造批准。

## 8. Preserved baseline invariants

26 个受保护 candidate 文件逐字节相同，包括四 YAML、install/common/start/Agent/deploy/fcctl、ground/protocol/velocity、attitude reference/wire/ownership/controller、原四 C++ 核心回归。精确 hashes 在 G_INVARIANTS.json。

Runtime::ack() 和 track() 在 0.3.1→0.4.0→0.4.1 三份源码中逐字节相同。ack SHA256=75d02bd3009d09ce2a2de429c2aae51b65314e9d3b625e4047731fc18144c090；track SHA256=bf2672b9ff3be4dcfa70b982bc9f83af21b72975fb367b64e2ffb996f8ee7854。target1/191、pending/freshness/watermark、from_external=false 接纳、IN_PROGRESS/超时、400 Arm/Disarm 顺序不变；原测试通过。

attitude profile 的 Mode+Attitude/no Trajectory planner 与 ownership byte 不变。安装保持创建独立 immutable release、仅构建后写 built-release，不自动换 current；显式更新/回退模型保留且更严格。pin/MESSAGE_VERSION 原假设不改。默认四 gate=false，attitude stage-two 推力/gain/limits 保留 -1 UNKNOWN sentinel，测试合成值不进入配置。

解压 baseline30个文件、candidate58个文件均与各自原 ZIP byte 相同；原 ZIP/TAR 规范化 root/./ 后 byte 相同；原 0.3.1→0.4.0 patch 也 replay 成功且 byte 相同。没有修改原版或用户真实 current，没有启动 Agent/controller/FC01 链。

## 9. Patch replay/archive verification

交付六项主文件：source.tar.gz、source.zip、vs_0.4.0.patch、本报告、SHA256SUMS.txt、PACKAGE_VERIFICATION.txt（使用合同规定完整文件名）。源码包包含 src/config/scripts/tools/tests/docs/README_zh.md/VERSION/package.xml；无 .git/build/install/log/cache/ELF/compiled test 输出。

新 patch 对 pristine 0.4.0 使用 patch --batch -p1 clean apply，replayed tree 与 delivered source 的全部路径、byte 和执行位匹配；ZIP/TAR 解开后也逐文件 byte/执行位匹配。具体计数、canonical manifest digest、原始 apply 输出在 PACKAGE_VERIFICATION.txt。SHA256SUMS.txt 对另外五个文件逐项核验；清单自身不做递归自哈希。

归档使用确定性文件顺序、固定 uid/gid 和 archive timestamp；临时编译输出在 /tmp，并由 test trap 清理。源码无需依赖本次 workspace 即可阅读和运行离线 suite；真实 build 仍需指定 pinned px4_msgs 及 Ubuntu22.04/Humble。

## 10. UNKNOWNs and unverified hardware behavior

- 新 0.4.1 的 Ubuntu22.04/Humble/colcon 实际 build/link、精确 pinned message 源码在线核对：UNKNOWN；旧 0.4.0 用户成功 build 不能代替。
- native AF_UNIX bind/listen/live listener/真实 console transport：UNKNOWN（本环境 EPERM）；对应测试保留供可运行环境执行。
- 新独立 reviewer：UNKNOWN（额度阻止任务运行）；当前有作者审查、源代码不变量和实际离线回归，未获得独立批准。
- 新版本 DDS/QoS/频率、real FC01 zero-thrust READY、graph count 的实际时序、PX4 Offboard loss/RC/manual takeover、shutdown 释放后的物理恢复：UNKNOWN。
- advisory lock 对 legacy/noncooperating restart 和恶意 same-UID filesystem actor 的完整竞态防护：UNKNOWN；已明确 human quiescence 必需。
- Orange Pi build/运行、推力校准/gain/limit、非零 thrust/Arm/Takeoff/物理飞行：UNKNOWN，未执行。

这些 UNKNOWN 不可由策略单测、type fixture 或旧现场证据填补。G2 如继续，使用独立 fake-only 范围和单独交付，不修改/弱化已冻结 G1。

## 11. Explicit statement: no real Arm/Takeoff authorization

本 release 不授权真实 Arm、Takeoff、非零推力、物理飞行或 Orange Pi 生产部署。未连接或控制真实 FC01，未写 PX4 参数，未使用 PX4 main 替换 v1.16，未安装或切换用户真实 release/current。所有 Arm/nonzero 的“允许”结果仅来自合成 policy/test sink，所有 current 安装/切换测试只作用于临时 fixture。
