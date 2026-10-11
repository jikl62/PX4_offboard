# FC01 M1 / 0.4.4-offboard-flight-dev-r3 — B 第三轮离线修复、遗留风险与 C 复审任务

日期：2026-10-09；**OFFLINE DEV ONLY / NOT CLEARED FOR FLIGHT / NO-GO**。

## 0. 溯源与授权

- 唯一实现基线：交接包 `INPUT_R2/FC01_M1_B_R2_SOURCE.zip`，SHA256 `82e9c2b5d8083f40bc8f7a9b1eadd5f0df67ef859dab8bd066c27f783e37a29f`；与上一轮 B 交付 ZIP 相同。
- 只在独立 R3 目录修改，未修改用户 R2 包及正式 current / built-release / release。
- C 第二轮报告 `FC01_M1_C_R2_INDEPENDENT_REVIEW_20261009.md`、C 的三个独立反例 `.cpp` 和对应运行 `.log` 逐项核对；反例代码原样复制（见 manifest）。本轮没有真实 FC01、ROS graph、Arm、Takeoff、飞控参数写入。
- 测试合成工况可能显式将 config.flight_actions_enabled / arm_enabled / takeoff_enabled / parameters_confirmed 设为 `true`，**并非生产配置**；`config/attitude_altitude.yaml` 中四项和 flight_enabled 都保持 `false`。

## 1. C 问题关闭矩阵（仅限离线源码与测试）

| ID / 等级 | 代码修改 | 可复核离线证据 | B 判断 |
|---|---|---|---|
| C06 / BLOCKER | `Core::descend`, `step(RequestArm)`：对已计划/已提交 Arm 的普通取消保留 `RequestArm` 受监管零推力心跳，不再 `fault -> Safety`；新 armed 状态后转 PoweredGround/Descent；无新状态/ACK 超时时禁止重发 Arm，并维持 guarded 输出；拒绝+新 disarmed 状态才进入 176 ground release。 | R2 raw stdout `heartbeat=0`；R3 `heartbeat=1`，原 C 不改；新增 LAND/SHUTDOWN/DISCONNECT/TAKEOVER、迟到 ACK/拒绝/超时、sink 双 publisher 断言。 | **已修复原触发条件，离线 GREEN** |
| C07 / BLOCKER | `Core::step(RequestDisarm)`：sink 后 land bounce 且 armed+Offboard 正常时转 `Descent`，当 tick 立即输出受限控制；后续接地等待旧 400 的 ACK + 新 status，防重复发 Disarm；ACK 被拒绝后重新获取 ground dwell；ACK 超时不重复 Disarm。 | R2 `heartbeat=0`；R3 `heartbeat=1`，仍 Armed+Offboard 进入 Descent；新测试 ACK 混序、拒绝、重复接地、post-commit detector freeze、ACK timeout。 | **已修复原触发条件，离线 GREEN** |
| R2-H01 / HIGH | `Core::recovery_responsibility()` 保留外部 fail-safe 状态声明；失去姿态、land、status、权限、Kill、控制循环失常时不能以任何日志宣称 PX4 安全接管。 | 原有 health、Kill、authority、最终 sink 用例；C 报告真实链路未验证。 | **软件声明保持正确；真实 failsafe 未关闭（FLIGHT-BLOCKING）** |
| R2-M01 / MEDIUM | `Runtime::Pending` 加 `command400_kind`、`stage`、`wire/status/land watermark`、`ack_stamp`、`state_verified / verified_status_stamp`；`confirm_attitude_command_publication` 只在匹配 kind 且 sink 成功时记提交；ACK 不代表终态；新 armed 状态可在 Arm ACK 丢失时释放旧 400 跟踪器供地面 Disarm 使用。Node 状态 JSON 提供对应审计字段。 | R3 `test_m1_r3_interleavings` 和原 C10；不同 ACK target、过期 ACK、重传、status-before-ACK、sink-denied、post-commit land 冻结。 | **新增离线证据，ACK 没有 param1 / nonce 的跨事务歧义依然 OPEN** |
| C01–C05、C09/C10 | 原实现与原始复现程序保留；C 的 C01..C10 在 `scripts/test.sh` 自动重放。 | C 原始源码在 R2 和 R3 用相同编译选项分别独立运行，日志见证据包。 | **无已知回归（synthetic）** |

## 2. Command 400 状态与安全边界

`Planned` 表示 Core 仅提议发送；`CommittedAwaitAck` 表示 ROS publisher/模拟 sink 调用返回且通过最终 guard；`AckAcceptedAwaitState` 不代表 PX4 已真正解锁或停桨；`StateVerifiedAwaitAck` 表示从新的状态中看到结果，但 ACK 缺失；`VerifiedTerminal` 同时看到 ACK 以及比提交时新的 PX4 状态。`Rejected/TimedOut/FinalSinkRejected` 单独审计。生产 `rclcpp::Publisher::publish()` 返回仅意味着本地调用成功，**不能证明 DDS/PX4 收包**。

C06 取消时：若 400 已提交且 PX4 仍在 Offboard，保持受最终 guard 审查的**精确零推力**心跳，禁止新任务；若 armed 状态变为 true 则进入受限地面撤动力（如离地则下降）。已提交 400 不被当作可以撤回。ACK 超时且当前仍 disarmed 并不能证明未来不会接收/执行，必须保留不确定状态。普通 LAND / 操作员退出不是 Kill。

C07 发出 Disarm 后：如果地面状态反转且 PX4 仍 armed+Offboard，原命令无法撤回，运行受约束 `Descent`；地面恢复不重复发送未完结的同一 400。终态需新的 status、**Disarm sink 之后的更新 land 样本**、地面仍确认和接受的 ACK。若 PX4 已 disarmed 但离地/地面失效，Companion 无法驱动已关闭的电机，进入外部 fail-safe 依赖；不能声称飞行仍可由自己控制。对于 ACK timeout，不自动重新发送一个可能重复的 Disarm。

**关键无解性**：ACK message 的 `command=400` 不含 Arm/Disarm 的 param1 / 唯一 nonce。以上关联通过 sink commit、timestamp watermark、固定目标 1/191、新 armed/nav/land 终态进行最大程度隔离，但 PX4 延迟的同 ID ACK 在极端异步混序下仍不能严格证明是哪一次请求。R3 不应宣称“已经从根本上解决 Command 400 ACK 关联”；实际行为必须由 SITL 及厂商固件验证。

## 3. 尚未关闭的飞行阻断（必须独立验证）

1. **原生 ROS 2 Humble build/test UNKNOWN**：当前环境没有 `colcon` 和 `/opt/ros/humble`；仅 C++ 标准编译和 ROS adapter fixture 语法检查通过，不能替代真正 ROS2/RMW/DDS 接口编译与运行。
2. **Offboard-loss、SIGKILL/断电、Agent 停止、RC 接管、Kill 真实行为 UNKNOWN**：Companion 掉电时无法持续发布；状态机中 `Px4Handoff` 仅是诊断，不是“PX4 已成功接管”。须只读检查 FC01 固件对应 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 等配置，并测试估计器/无 XY 状态下的模式可用性。PX4 1.16 文档要求 Offboard 活性信号 >2Hz，失联后行为依赖配置：https://docs.px4.io/v1.16/en/flight_modes/offboard。
3. **真实 PX4 Commander 对 in-air Disarm、长延迟 ACK、重复 400、接地抖动拒绝的行为 UNKNOWN**：合成 sink 不能证明飞控实际拒绝空中 Disarm；这是高风险的独立阻断项，需 SITL 优先、无桨台架后续单独授权。
4. **动力学/控制稳定性 UNKNOWN**：hover、推力映射/电调响应、悬停或垂直下降速度、接地检测、陀螺仪姿态有效性、掉电保护和电池压降缺乏真机证据。
5. **保持 guarded 输出的前提严格**：姿态、Z/VZ、land/status、Failsafe、Authority 失效后软件只能 fail closed，不能凭代码保证继续健康控制；真实上必须由已验证 PX4 后备行为兜底。
6. **确认不等于接管安全**：观察到 PX4 不在 Offboard / armed=false 不等于 RC 已经可靠控制，尤其在缺少位置估计的情况下。代码不允许自行写 PX4 参数。
7. **ACK 关联不可完全证明**：同一 ID 400 无 request nonce，极端迟到 ACK 跨事务无法纯靠 ACK 时间戳完全消歧；应对 PX4 实现作独立审查及仿真复现。
8. **C 第三轮独立审查未完成**：B 自测不能作为飞行放行签字；独立复审和真实 ROS/SITL 证据缺失前 NO-GO。

## 4. 构建和测试证据等级

- 全量 `bash scripts/test.sh`：见 `R3_FINAL_NEW_SENSOR_TEST.log`，实际退出码 `0`；原 C++ 15 个加 R3 新增 1 个=16 个测试可执行文件，原 C++ 独立反例 3 个另行编译运行，Python 18 项全部成功。
- C 的原 C06/C07 代码完全不改：R2→R3 原样 g++ `-std=c++17 -Wall -Wextra -Werror -pedantic` 编译运行。R2 gate 退出 `1`，R3 gate 退出 `0`（见 `RED_GREEN_EXIT_CODES.txt`）。原 C01–C05、C09/C10 同样保存 R2/R3 stdout。
- 真正 `colcon build/test` = **UNKNOWN**，ARM64 本机 = **UNKNOWN**，真 ROS 回调/DDS = **UNKNOWN**，FC01 接入 = **未执行**。
- `patch --dry-run` 和实际 patch 回放，并对 409 个 R2 原文件、260 个 px4_msgs 哈希完全一致核验，结果以独立交付完整性报告为准。

## 5. 给智能体 C 第三轮复核指令

1. 验证 R2 固定源包 SHA256，R3 ZIP CRC、逐文件 hashes，patch -p1 正向重放哈希完全相同；确认原 409 个文件没有丢失，260 个 px4_msgs 未修改，四重门控和 `flight_enabled` 默认为 false。
2. 编译执行 **原样未修改** `tests/c_independent_repro/C_M1_COUNTEREXAMPLES.cpp`、`C_R2_NEW_INTERLEAVINGS.cpp`、`C_R2_SLEW_AND_ACK.cpp`；比较交付 RED/GREEN 原 stdout，不允许只读 summary 表格。
3. 审计 `Core::descend/step(RequestArm,RequestDisarm,Descent,Landed)` 与 `Runtime::track/ack/confirm_attitude_command_publication/step` 及 Node `tick -> final attitude sink -> command publish -> commit` 的实际调用链。特别验证 **ACK 丢失但新 armed 状态来了** 时后续 400 Disarm 不能异常阻塞输出；**Disarm 发出后新 land 翻转** 不得立刻断健康心跳。
4. 检查 `request_operator_takeover/disconnect/begin_shutdown` 与 session/shutdown guard 的同步关系；synthetic zero output 必须通过最后两个 publishers 的决策。
5. 注入虚假 target、重复、倒退/冻结 stamp、ACK result=5、Command 400 跨事务重复及极晚 ACK；特别检查真实 PX4 1.16 Commander result/target 字段。该关联仍有明确 OPEN 风险。
6. 在隔离的 Ubuntu 22.04 + ROS Humble 本机执行 `colcon build --symlink-install`、`colcon test` 和 `colcon test-result --verbose`，记录完整日志、exit codes 与使用的 `px4_msgs` 版本。后续 SITL、无桨、真实 FC01 全部需要新阶段授权；此交付不得作为真机带桨飞行许可。
