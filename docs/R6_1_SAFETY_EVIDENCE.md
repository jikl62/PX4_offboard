# R6.1 安全回归与证据边界

本轮未连接 FC01、未操作真实飞机。用户报告的 Disarmed 模式进出与传感器状态作为当前接口约束，原始 Armed/动力/后备证据未提供。

## 可证明的代码性质

| 条件 | R6.1 动作 | 不应误认的证据 |
|---|---|---|
| 健康 Armed+Offboard，普通下降超时 | 原 RecoveryDescent，继续受限反馈下降与双 sink | 不保证已实际下降或落地 |
| 恢复预算耗尽，仍持有健康 Offboard | 原截止不续期，逾期锁存；原控制与最终 gates 每周期重检 | 计时耗尽不是实际接管 |
| 21 ACK 接受/拒绝/丢失，PX4 仍 Offboard | 保管继续，包括截止之后；不重试 21，不发无证据 Disarm | ACK 不是 flight task 执行证明 |
| 当前 XY/VXY 无效 | 允许原姿态/Z/VZ 路径；原 Land21 完整本地位置 gate 拒绝 | 无 XY 时不假定 Land/Position 后备可用 |
| 新鲜 PX4 status 已退出 Offboard | 尊重原模式退出与 Authority 锁存，停止 Companion 输出 | 模式退出仍标 CONTROL_UNVERIFIED；没有实际 task 成功证明 |
| 新的连续 1Hz land + 原高度/速度/at_rest 满足 | 原 ramp-to-zero、ground dwell、一次 guarded Disarm、独立 state-only 终态、地面释放 | 一个 landed pulse、旧/重放 stamp 不构成落地 |
| stale/reset/tilt/jump/Authority/Kill/battery/饱和/stall/发布异常 | 原失效闭锁保留，禁止无证据 routine Disarm | 不宣称此时 Companion 还能健康下降 |
| SIGINT/TERM/控制台断连 | 原 LAND 请求；禁止新任务；健康保管跨过截止 | Runtime OS fixture 不等于真实 rclcpp/PX4 验证 |
| SIGKILL/crash/供电断开/Agent 丢失 | 不能保证继续本地发布；仍需经验证独立后备 | 本轮没有证明物理安全交权 |

“guarded”说明每次输出依赖原安全合同，不说明无限持续推力安全。截止后总时间不受该计时器终止，持续无进展必须由现场责任方依据经验证后备/接管处理；本版不会伪造这样的证据或自动改参数。

## 本轮离线回归

| 检查 | 证据与范围 |
|---|---|
| 原 C 扩展门禁 | 原 R6 `38.05s / exit1 / RED`；R6.1 `exit0 / GREEN`；测试源字节不变 |
| 原 C-S10 timeout / shutdown | 分别 exit0；原 460 tick 正向断言不变 |
| 更长时窗 | 三种中断情形各 600 秒虚拟时间、1Hz detector、无 XY/VXY；每 tick 验证 target 和 heartbeat sink 调用，推力/slew/固定 deadline，无额外命令 |
| 逾期后故障 | 13 类原 fatal 条件 + 四种最终 guard 否决；无 unsafe sink、无 routine Disarm |
| 延迟接地 | 新 1Hz 正向样本与驻留、单 pulse 拒绝；预算之后完成一次 state-only Disarm 和原地面释放；保留跨事务 ACK 歧义 |
| Command21 | 真正 boundary 和 final commit；接受/拒绝/缺失 ACK 均跨完整预算，不重试；新 mode 与 ACK 分开 |
| 完整脚本 | `scripts/test.sh` 完整 exit0；22 个 C++ regression executables，Python30 = 28 PASS / 2 SKIPPED。两项 AF_UNIX 环境限制不算 PASS |
| 历史反例 | C11/C12 漏洞肯定式探针预期 abort 是诊断；R4 正向安全断言才是成功判据 |
| 实际 Node 编译 | actual source 的 type/API fixture syntax compile PASS；不是 ROS Humble native build/link/DDS 结果 |
| 完整性 | 原 C/R4 tests、235 个 pinned px4_msgs、Runtime/ACK、Authority、控制器、最终 guard/boundary 字节相同；所有 YAML 解析值相同 |
| 补丁重放 | 精确 R6 完整副本上 `patch --batch -p1`，文件集合与内容逐一等于 R6.1；各归档 CRC 与 SHA256 验证 |

首轮完整脚本被旧 B 的 `ascend_timeout_is_supervised_then_bounded` 终态断言中止，exit134，日志保留；只更新该过时预期，原 C 和 1Hz 证明条件不修改。新正向回归先在 R6 得到预期 RED，然后验证修复 GREEN。

本环境为 Ubuntu24.04.3 x86_64，无 `/opt/ros/humble`、无 colcon；不能提供 Ubuntu22.04/Humble 原生构建证据。当前版本 default flight/Arm/Takeoff/parameters gates 关闭；RC/autonomous/fallback confirmation 不变，动力参数 unset。

## 当前判定

- 特定“恢复计时耗尽主动停止健康 Armed+Offboard 输出”实现缺陷：离线肯定式回归已关闭。
- 离线源码：可继续独立 C 复审；本轮开发代码复审不代替正式 C 发布签署。
- 用户报告的真实 Disarmed Offboard 进出：保留，不重复实现或要求重做。
- 当前源码 Humble 原生构建、真实 Armed 运行、推力/电池、接管/Kill、失流后备用 task：NOT VERIFIED。
- 完整有桨自主起飞→短时高度保持→降落：尚不能放行。本版不签署恢复 P0 的全部物理场景已关闭。

## 官方依据（2026-10-10 核对）

- [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)：ROS2 `OffboardControlMode` 心跳与设定值分离，必须持续满足心跳要求；失流触发动作依赖配置与可用能力。
- [PX4 v1.16 Land](https://docs.px4.io/v1.16/en/flight_modes_mc/land)：要求有效本地位置；模式名/Command21 ACK 不能替代厂商固件实际可运行任务证据。

本地 publish 成功只是本地提交，不是 PX4 已收到、已消费或物理控制成功。source/evidence 校验结果以交付日志和 manifest 为准。
