> **R4 DEV CANDIDATE / FLIGHT NO-GO**（2026-10-09）。当前契约与风险以 `docs/M1_R4_ACK_ISOLATION_AND_REMAINING_RISKS.md` 为准；历史文档只用于版本追溯。

# FC01 M1 `0.4.4-offboard-flight-dev-r4` — 第四轮离线增量修复

**直接基线**：交接包 R3 完整源码，ZIP SHA256 `056ecc37d3b7801afa3a2c87413ce333781c812948c61c54de9f6fcc6982b620`。R4 保留 R3 全部文件和原有控制框架；`package.xml` 保持 0.4.4，迭代号见 `VERSION`。R1/R2/R3 及更早文档是历史证据，不能替代 R4 验收。

**本轮行为**：Command 400 一旦复用，ACK 显式标成 AMBIGUOUS，保留原始结果和前事务历史；不将旧 Arm 成功 ACK 认作新 Disarm 确认，也不让无法归属的失败 ACK 引发重发。新鲜可信 PX4 状态与接地证据可完成 state-only 地面恢复；最终 sink 禁止同一任务再次提交常规 Disarm。安全地面 Authority reset 保留可用并保留歧义；同一 Runtime 已有 400 commit 后的新 Arm 明确要求外部确认与安全地面重启，不自动清除歧义。

**执行与构建边界**：仅离线源码开发与合成 sink 测试；没有连接 FC01、真实 Arm/Takeoff/Disarm 或参数写入。五项飞行默认门仍为 false。R4 本环境真实 ROS Humble colcon build/test UNKNOWN；类型/API fixture 不是原生构建。完整原始回归日志随交付提供。

**独立反例**：C01–C07、C09–C12 原始反例保持原样。C11/C12 的原始 assert 检查旧漏洞存在，R4 在这些 assert 处停止是诊断证据；`tests/test_m1_r4_ack_isolation.cpp` 的新肯定式安全断言才是 R4 的成功判据。交接材料没有单独 C08 原始用例。

## 权限与默认配置

进程启动一律 UNSET；配置没有自动选择模式的入口。authority 是 companion 安全策略，不是 PX4 nav_state。

| 模式 | Arm / 非零推力所需权限条件 |
| --- | --- |
| UNSET | 永远拒绝；仅允许健康、disarmed、exact-zero 第一阶段准备 |
| RC_TAKEOVER | 物理 RC 来源、信号和消息新鲜、RC/开关均至少两帧且原始时间戳推进、无 RC lost/failsafe、PX4 manual-loss flags 正常、唯一 RC 发布源、mapped Kill OFF 开关遥测，以及 rc_takeover_confirmed=true |
| AUTONOMOUS_NO_RC | 不要求 RC，但必须 autonomous_failsafe_confirmed=true |

交付中 flight_enabled、flight_actions_enabled、arm_enabled、takeoff_enabled、flight_parameters_confirmed 五项开关均为 false；继承配置 rc_takeover_confirmed=true、autonomous_failsafe_confirmed=false 原样保留。所有原飞行数值和 UNKNOWN sentinel 保持不变。选择模式从不确认这些契约，也从不改 PX4 参数。confirmation 是 startup 固定配置，必须来自另行审查和验证；本包没有在线打开它们的命令。

所有飞行门、battery/preflight、attitude/Z/VZ health/freshness、ownership、Offboard、ground reference、envelope 和 session/lease 条件仍适用。最终 Arm 和非零姿态 sink 都重算权限，不信任上游预先算出的 authority_ready。manual_signal/manual_ready 仅是保留的历史观测，不构成新权限证明。

authority_ready 表示所选路线的权限条件满足且没有已观察到的接管/故障阻断；选择后可以在未锁定时显示 ready。它不表示四个飞行门、完整健康检查或起飞许可。Kill/显式接管/failsafe/已建立 Offboard 的丢失事实，在下一次 Core step 前也使 readiness=false。原始 RC/开关数据需要在既有 flags_timeout 窗口内观察到两次推进；首帧、重复 raw timestamp 或 stale gap 后的首帧均不构成活性证明。这也不证明校准或物理接管，后者仍依赖独立确认。

## 控制台接口

| 控制台文本 | 协议操作 | 行为 |
| --- | --- | --- |
| authority status | AUTHORITY_STATUS | 返回模式、锁、readiness、confirmation 和稳定原因 |
| authority rc | AUTHORITY_RC | 选择遥控接管路线 |
| authority autonomous | AUTHORITY_AUTONOMOUS | 选择自主无遥控路线 |
| authority takeover | AUTHORITY_TAKEOVER | 活动任务仅提出接管请求；armed+Offboard 时先执行受控下降/地面撤动力，**不立即锁存已交权** |
| authority reset | AUTHORITY_RESET | 确认安全地面、输出停止、无未结束事务后回 IDLE/UNSET，不开始准备 |

选择/reset/takeover 只允许 live interactive console，one-shot --command 在 socket 连接前拒绝；管道输入不能替代 TTY。服务端仍使用基线的可信同用户 INTERACTIVE+PING 模型，不宣称对同 UID 自定义恶意客户端提供强认证。

选择仅在 IDLE、fresh/unique/healthy、disarmed、landed、at_rest、满足既有静置窗口、实际和生成的 target/heartbeat 都停止、无 pending 176/400、无故障时接受。准备成功进入 SELF_CHECK 就锁定模式，覆盖 PRESTREAM、Offboard、Arm、飞行、落地和 recovery。成功 Disarm 后仍需显式 ground reset，才能再次选择。第一阶段 UNSET 准备后同样须先释放、reset，再选择飞行路线。

## 接管、失联与恢复

观察到 Offboard 后，实际 PX4 mode departure、Kill、TERMINATION、PX4 failsafe、critical failure 和 Offboard signal loss 依照 fail-closed 分支中止 Companion 输出（失控时是否安全由尚未验证的 PX4 failsafe 决定）。普通遥控开关变化/显式接管若 PX4 仍回报 armed+Offboard，**不再将请求当作实际交权**，而是进入受限下降或 PoweredGround 撤动力流程，持续提交经最终 sink 审核的控制输出；模式确认后再停止 Offboard 输出。开关六项位置或 PX4 switch_changes 计数任一变化均视为干预；计数跳变能识别中间位置消息丢失，回绕/重置也保守释放。RC 模式活动飞行失去 RC/开关遥测也释放。健康输入恢复不重试 Offboard。

Kill ON 观测保留，直到收到更新、新鲜且 mapped OFF 的 raw sample；NONE/UNKNOWN/重复 sample 或 stale 不能清除 Kill。接管后的 reset 要重新具备 live lease 和安全地面，无 pending Arm/Disarm、无输出。普通安全/发布故障只能安全地面 restart；这一重启要求独立于 takeover latch，先接管再故障也不能通过 reset 清除。未决 command-400 不会被 reset 丢弃，必要时另行安全地面重启。

保持基线的 healthy controlled descent：session 断线/lease expiry 后，健康且权限仍完整的 DESCENT/LANDED 可继续现有受控降落；RC loss、kill/接管优先释放。shutdown 的 final boundary 仍禁止不属于受控下降 / PoweredGround / Landed / RequestDisarm / 安全 Release 的输出。本包不增加自动强制 Disarm，不实现 companion 手动摇杆，不改变 PX4 native takeover。

RC 读取 /fmu/out/input_rc 与 /fmu/out/manual_control_switches，仅订阅。若实际 DDS firmware 没有暴露这些主题，则 RC readiness=false。自主路线与 UNSET stage-one 不因这些主题缺失而被拒绝。是否发布、校准、Channel5 Kill/Channel6 Arm、stick override 和真实物理效果均 UNKNOWN。

现场已有 COM_RC_IN_MODE=4、COM_RC_OVERRIDE=1 不证明 Offboard 遥控接管：v1.16 中前者禁用 stick；后者是 bit0，Offboard override 使用 bit1。这里是诊断，不提供参数修改或飞行放行。自主失效保护也尚未确认，尤其无 XY 定位时不能从失联动作名称推断物理安全。

## 离线验证

```bash
bash scripts/test.sh
bash scripts/test_authority_harness.sh /tmp/authority-results.json
```

测试只执行纯核心、类型/API fixture、临时文件系统或 fake 系统工具，不连接 FC01/Agent、不启动真实 ROS 控制链、不写真实 current。R4 共 17 个 C++ 测试程序；实际 ROS adapter 的 fixture syntax 和 Python suite 只验证离线行为。Python 18 项的 PASS/SKIP 数量以本次原始日志为准；受限的原生 AF_UNIX 用例明确报告 SKIP。原生 AF_UNIX 在本执行环境受限，明确 UNKNOWN。

新 harness 直接 include 本 release 的 Runtime/publication_boundary，24 场景重复执行 JSON byte-identical；包含仅合成 sink 的 Arm/nonzero 正向策略证明、拒绝路径和接管不重入。它不模拟动力学、真实 graph 时序或 PX4，不能替代真实 DDS/FC01 集成。原 frozen G2 39/39 的复跑仅验证原 0.4.1 harness；新候选证据见 docs/I_*。

## 构建与更新边界

install.sh 保持原文：构建唯一新 release，成功才写 built-release，永不自动切 current、启动 Agent/node/service。历史 0.4.2 说明仅供追溯；R4 的 Ubuntu22.04/Humble build/link、DDS/topic 集成和 Orange Pi 本次均 UNKNOWN。本环境的 fixture 编译不是实际 ROS build。

socket lifetime lease、未知 liveness 拒绝、stale-looking socket 不自动删除、service UNKNOWN 拒绝、临时 symlink 和原子 replace 保持原文。不得删除永久 .lock；legacy/non-cooperating starter 的竞争仍须独立 quiesce，advisory flock 不能约束它们。

0.4.1 已有真实 build/stage-one 证据，见 docs/i_contracts/02_VERIFIED_FIELD_BASELINE.md；它不自动成为 0.4.2 证据。历史 D/F/G 文档及 README_0.4.1_HISTORICAL.md 是基线历史，不是新 release 验收。当前报告、验收矩阵、测试和 package verification 是本次证据。

## R3 修复版本（2026-10-09）
本版本仅在隔离离线环境中研究已提交 Command 400 的取消及 Disarm 后地面状态反转；真实 PX4 Commander 拒绝空中 Disarm、ROS Humble 构建、RC/失联接管及动力参数均未完成验收。

## R4 修复版本（2026-10-09）
当前版本保留跨事务 ACK 的不确定性、状态证据终态与地面恢复路径；完整协议语义、审计字段、一次任务的 400 重用、第二次 Arm admission 限制、测试解释与 C 第四轮复审要求见 `docs/M1_R4_ACK_ISOLATION_AND_REMAINING_RISKS.md`。本轮没有实机或 R4 原生 ROS 验收。
