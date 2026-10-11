# FC01 M1 R4 — Command 400 隔离、恢复契约与残余风险

版本：`0.4.4-offboard-flight-dev-r4`，2026-10-09。仅独立 DEV 离线开发。
状态：DEV CANDIDATE / FLIGHT NO-GO；不是智能体 C 的独立验收结论。

## 基线与范围

唯一开发基线是交接包的 R3 完整源码，原 ZIP SHA256：
`056ecc37d3b7801afa3a2c87413ce333781c812948c61c54de9f6fcc6982b620`。
原始 413 个文件全部保留；原 RC、DDS、Authority、Command 176/400/21、发布策略、旧 profiles 与固定 `px4_msgs` 未重建。
实际文件数、变更清单与逐文件验证见本次交付清单，不沿用 R3 的测试结果充当 R4 证据。

没有连接真实 FC01，没有发送真实 Arm/Takeoff/Disarm 或写入 PX4 参数。
`flight_enabled` 和四项 flight gates 的交付默认值仍全部为 false；原 UNKNOWN 动力数值与 authority confirmation 默认值保留。

## 根因与处理策略

Command 400 的 `param1` 区分 Arm 与 Disarm；固定 `VehicleCommandAck` 没有原始 `param1`，也没有唯一事务 nonce。
单调时间戳、目标检查与发送水印只能排除部分 ACK，无法证明在新 Disarm 之后生成的旧 Arm ACK 属于新事务。
这既适用于旧 Arm ACK 缺失，也适用于旧成功 ACK 的重复应答。

R4 沿用现有状态机，新增以下保守规则：

1. 首次 400 的 Request、最终 sink commit、ACK 观测与 PX4 状态分别记录；本地 generation 只是审计序号，绝不当作 wire nonce。
2. 同一 Runtime 一旦复用 400，`ack_ambiguous=true`。不因旧 ACK 成功、时间流逝、超时、普通 LAND 或 authority reset 而解除。
3. 复用后的每个合法目标/水印 ACK 保留原始 result、时间与观测次数，标为 `AMBIGUOUS_400_ACK`。不设置该 Disarm 的 `accepted/rejected`；无法归属的失败不授权重试。
4. 原 Arm 跟踪条目归档到 `command400_history()`；`previous400_unresolved` 明确保留前事务未取得相容 ACK/状态证据的事实。
5. Disarm 的实际最终 sink commit 后，一次任务内不再发送第二个常规 Disarm。最终发布边界重读实际 `Pending.published`，拒绝重放。
6. 解除恢复只能依赖新的可信 PX4 状态与地面证据。审计写为 `PX4_STATE_ONLY_TERMINAL_ACK_UNATTRIBUTED`，不写 `ACK_AND_PX4_STATE_VERIFIED`，也不证明该 Disarm 是状态变化的原因。

ACK 的物理归属仍是 AMBIGUOUS/UNKNOWN。本次修复的是软件接受歧义时的危险决策，不宣称为协议补出了 nonce。
原 `command_timeout=5s` 没有延长，没有新增固定 sleep、延迟清队列或 Commander 顺序假设。

## ACK 结果与审计语义

| result | 协议名称 | 首次 Arm 的既有处理 | 复用 400 的处理 |
| --- | --- | --- | --- |
| 0 | ACCEPTED | 记录 ACK；仍需新 Armed 状态 | 仅观测，归属不明 |
| 1 | TEMPORARILY_REJECTED | 记录临时拒绝；保留恢复 | 仅观测，不重试 Disarm |
| 2 | DENIED | 记录拒绝；保留恢复 | 仅观测，不重试 Disarm |
| 3 | UNSUPPORTED | 记录不支持；保留恢复 | 仅观测，不重试 Disarm |
| 4 | FAILED | 记录执行失败；不假定必定未 Arm | 仅观测，不重试 Disarm |
| 5 | IN_PROGRESS | 尚无终态；允许后续首次 Arm ACK | 仅观测，不完成/重试 |
| 6 | CANCELLED | 记录取消；仍核对新状态 | 仅观测，不重试 Disarm |
| 其他 | UNKNOWN_RESULT | 明确未解决，不能冒称拒绝 | 仅观测，不完成/重试 |

同 timestamp 的重复应答不重复计数；新的 timestamp 仍不能消歧。
ACK target 不匹配、发送前/最终 sink 拒绝的 ACK 不授权动作。`from_external` 保留原厂消息语义，不能当作关联 token。

首次 Arm 失败 ACK 与实际新 Armed 状态矛盾时，执行监督恢复，标签为 `PX4_STATE_EVIDENCED_ACK_CONTRADICTORY`。
已观测 IN_PROGRESS/未知结果而状态先到时，标签为 `PX4_STATE_EVIDENCED_ACK_OBSERVED_UNRESOLVED`；确实没有 ACK 才写 missing。
首次 Arm 的既有 ACK 匹配仍基于单次本地事务、固定目标和唯一发布源假设；它不是跨进程、跨发送者的密码学归属证明。

## 新鲜状态、地面与最终 sink

状态终态候选必须满足：实际 Disarm 已提交；status 比提交水印和事务 baseline 新；land 比提交水印新；原始 arming enum 明确为 DISARMED；system/component 正确；唯一发布源；新鲜态度、local、status、land、failsafe；无 Kill、failsafe、时间回退或 ownership 冲突；合法 authority/四重 gates；有效高度参考；近地高度/垂速、landed 和 at_rest。

本轮适配层只承认固定 schema 的 `ARMING_STATE_DISARMED=1`、`ARMING_STATE_ARMED=2`。
未知 enum 0/3/255 等令 status 不可信，不把“不是 ARMED”当成“确认 DISARMED”。
纯核心历史 fixtures 的 `arming_state_known` 默认 true 表示调用者已经显式提供二值状态；生产 ROS adapter 始终从原始枚举设置它。

Runtime 先传入状态候选，再要求同一拍 Core 真正通过参考重置、控制循环 stall、估计器跳变和 envelope 检查并到达 SafeDisarmed，才记录 Disarm `state_verified`。
Core 进入 Safety 时不得同时将本拍标成安全终态。`state_verified` 是带时间戳的历史观测证据；后续实时 armed/nav/land 与新故障仍独立检查。

姿态/Offboard 两个 sink、Arm/Disarm/release 的既有最终审核仍存在；两次 ROS 发布不构成原子 DDS 事务。
`rclcpp::publish()` 返回仅说明本地 sink 完成，不证明 PX4 收到、执行或停止电机。

## 恢复与人工边界

- Arm ACK 丢失但取得新的 Armed 状态：不重复 Arm；取消、断连、shutdown 或超时后进入原受监督地面撤动力/下降路径。
- 首次安全地面 Disarm 仍可提交：必须经过限速撤动力、连续新 ground detector 样本、近地 Z/VZ 及最终 sink。不是永久禁止所有 Disarm。
- Disarm ACK 缺失/不可归属而仍 Armed+Offboard：保持健康、受最终边界审核的零推力或原受控下降输出；不常规空中 Disarm、不把 LAND 当 Kill。
- 触地反弹：保持原受监督下降与重新获取地面证据；待实际 Disarmed+新安全地面状态后才结束恢复。旧失败 ACK 不授权重发。
- 5s 超时：标记未知，继续受守卫恢复；不是命令未执行的证据。原下降超时、健康链失效、Kill、实际 PX4 mode departure 等仍按原 fail-closed 路径处理，实际后备安全性 UNKNOWN。
- 安全地面退出 Offboard、输出停止之后，Authority reset 恢复可用；它保留 ACK 歧义、history 与本地 generation。
- 本轮同一 Runtime 只允许一次新的 Arm admission。已有 400 commit 后新 TAKEOFF 明确返回 `COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART`。这不会撤销当前任务的首次地面 Disarm。
- 本包没有在线“清除歧义”、强制 Disarm 或自动部署命令。控制台 status/authority status 提供审计，LAND/接管请求只进入既有恢复。需要人在外部核对真实 armed/nav/land、完成原模式接管与安全地面停机；重新启动还必须证明旧命令队列/发送者已独立 quiesce。固定等待或单纯进程重启不是协议消歧证明，也不构成下一次飞行许可。

附带修复：拒绝 Arm + 新 disarmed ground 后的 ReleaseOffboard 不再被下一拍“飞行中异常 Disarm”守卫误杀；release 自身的健康、未解锁、land/at_rest 与 ACK/新 STAB 检查继续生效。

## 测试解释与保留

`tests/c_independent_repro` 的 C01–C07、C09/C10 原始三份反例不修改；C11/C12 原始源码与交接附件 SHA256 相同。交接材料没有单独编号 C08 的原始反例，不虚构该用例。
原 C11/C12 的 assert 用于证明旧漏洞存在。R3 原样执行 exit 0；R4 原样执行在漏洞存在性 assert 处 SIGABRT(-6)，两例各自原始 stdout/退出码保留。这个 abort 不是 R4 的成功证明。

新的 `test_m1_r4_ack_isolation.cpp` 使用真实 Runtime/Core/最终边界与本地合成 sink，给出肯定式安全断言：ACK 不误归属、失败不重复 Disarm、正确终态、实际 sink 次数和逐拍心跳。它在未修改 R3 上先失败，R4 才通过。
矩阵覆盖全部枚举、丢失/重复/晚到、状态先于或晚于 ACK、矛盾状态、错误 target、sink 拒绝、freeze/regression/bounce、重复 LAND、断连/shutdown/接管、安全 reset、未知 arming enum、参考 reset 和 stall。

B 的旧测试中与 R4 安全契约冲突的 Disarm ACK 接受/失败后重发预期已更新；C 原始反例没有修改。C01 的 B 断言 fixture 改为显式 require+confirm 最终 commit，避免把历史隐式发布再次当作首个真实 sink。全部差异可在 patch 中审查。

## 外部阻断与证据等级

| 项目 | R4 当前证据与限制 |
| --- | --- |
| C11/C12 软件决策 | 新肯定式断言通过；原始漏洞存在性断言不再成立 |
| 原 C 反例/完整回归 | 实际原始 stdout、退出码在交付日志；不能只看退出码解释旧反例 |
| ROS adapter | 类型/API fixture 语法编译；不是实际 ROS link/DDS |
| 本环境 native ROS Humble colcon build/test | UNKNOWN / NOT RUN：未安装 Humble 和 colcon |
| 任务书提到的 R3 x86_64 构建 | 仅 R3 外部背景，不能替代 R4 原生构建 |
| 原生 AF_UNIX | 本环境受限用例明确 SKIP / UNKNOWN，见本次实际日志 |
| 真 Commander ACK 生成/排序、模式时序 | UNKNOWN，未运行本轮 SITL |
| 重启前清队列/不同发送者共享 source ID | UNKNOWN，无法靠现有 ACK 字段证明 |
| Offboard loss、Agent/进程崩溃、Companion 断电 | FLIGHT BLOCKER；软件状态机无法在进程停止时继续心跳 |
| FC01 厂商固件、RC/Kill/空中 Disarm 拒绝 | UNKNOWN，未连接硬件 |
| 动力参数、空中高度控制、真实接地检测 | UNKNOWN，未验证 |
| ARM64 原生 | UNKNOWN，未构建 |

下一阶段由 C 对确切 R4 SOURCE/patch/hash 与日志做第四轮只读审查；另在 Ubuntu 22.04/Humble 原生构建与测试 R4。
SITL/Commander 验证应检查迟到结果、丢失/重复 ACK、状态反转、DDS 回调次序、接管/失联保护和地面终态；本轮不启动该真实 ROS/SITL 控制链。

## 官方依据

本轮已核对以下 PX4 v1.16 官方文档和包内固定消息，结果名称与字段一致：

- https://docs.px4.io/v1.16/en/msg_docs/VehicleCommandAck
- https://docs.px4.io/v1.16/en/msg_docs/VehicleCommand
- https://docs.px4.io/v1.16/en/flight_modes/offboard

由 ACK 字段缺少原命令参数/nonce 推导出的“不能普遍唯一归属”是本次代码设计判断；不是文档声称 Commander 必定产生本合成交错。
Offboard 的真实失联动作取决于飞控配置与可用控制能力。保留 FLIGHT NO-GO，不把 Companion 的 Safety/Px4Handoff 名称当成硬件安全结果。
