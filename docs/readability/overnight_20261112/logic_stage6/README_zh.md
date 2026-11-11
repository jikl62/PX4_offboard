# 队列 6/7：异常路径审查与可验证缺陷

基线为干净的 **`1e824f883fe6255f8d265b1cece339a83594b059`**，分支 `readability/fc01-chinese-20261111`。本轮源码交付点 **`212f477c67a036f302a604099a492f8be80c7f17`**，报告另作本地提交。此前历史全部保留，没有合并、推送、部署或真实飞控动力操作。

重新从实际控制/异常路径找反例，没有只依据已有注释或测试结论。完成两项普通工具修复；确认四项涉及地面准入、控制循环工作量或监督器生命周期的问题，**均未改入生产**。三项有独立候选补丁，一项只有复现和暂不修补的理由。没有足够证据新确认 P0/P1，也不能据此宣称不存在。

## 已修复：两个普通工具错误

| 等级 / 本地提交 | 可复核原失败 | 修复及实际结果 |
|---|---|---|
| P2 / `0c59198` | 被动 `navigation_collect.sample_ros` 的 init 在保护区外；destroy_node 抛错会跳过 Context shutdown，业务首错可能被后续清理错误替换。新 15 方法在基线出现 **7 FAIL、1 ERROR**。 | 将 init 的取得责任纳入保护区，依次尝试 Node 与自有默认 Context 清理；调用前已活动的 Context 不关闭。保留业务异常对象、原 cause 和后续清理错误，清理失败仍可见。原 9＋新 15＝**24/24**；含 12 次重复假资源周期、3 组正常/部分报告轨迹差分。另在隔离 Humble 域完成真实订阅-only的 **3 次正常启停**。 |
| P2 / `6c4cc7d` | DeploymentLog 已打开日志后，BEGIN 写失败使构造不能完成；finish 的正文/摘要失败也会跳过文件 close。新 8 方法在基线 **7 FAIL**。 | 构造中途失败自行承担文件关闭；最终写入失败仍尝试关闭自有文件。原写错优先，close 次生错误连入异常链；没有写成功标记来掩盖故障。**8/8**，含 10 次真实临时文件创建/结束循环、磁盘满、权限失败、停止异常和关闭错误。 |

以上“释放”指可靠地尝试释放；若 destroy/shutdown/close 自身失败，不能保证操作系统资源已经释放。默认 ROS Context 所有权是本工具单次串行调用合同，不是多调用者的并发租约。DeploymentLog 的新关闭分支只管理日志，不停止/启动子进程、Agent 或服务；`offline/online/main/record/step` 完整 AST 与原基线一致。

`212f477` 保存 IPC/时钟异常合同测试与历史 AST 断言的精确更新。新增 C++ 测试覆盖分帧/合帧、NaN/Inf/浮点溢出、序号重复、处理器异常、断连/重连、旧数据续鲜与时钟回退等，共 **122 条运行断言**；没有把已发现的预算问题写成“正确行为”。

## 需人工审核：生产未采用的四项问题

| 编号 / 等级 | 已证明的事实与根因 | 候选、未证明内容与审查要求 |
|---|---|---|
| **C1 / P2 地面时间证据不足** | 实际 Runtime/Core＋原生产值夹具：3 条不同 land 样本只跨 **0.10 s**；随后冻结 **1.90 s**，在 **2.05 s** 地面参考合格并进入 `RC_READY_WAIT_ARM`。源/接收年龄仍小于 2.5 s，但超过独立间隔 1.25 s。根因是用主机 `now-ground_since` 凑窗，land gap 只在下一条新 land 到来时检查。原同驱动失败两处：短突发合格、既有合格证据冻结后保留。 | `candidates/S6_C1_ground_burst.patch` 要求最后新 land 观察跨足窗口，且距当前不超过既有独立间隔。四轨迹通过：短突发拒绝、健康 1 Hz READY、过期撤销、完整新窗恢复；原 RC 三个相关回归通过，ASan/UBSan 通过。**改变 READY/资格及锚点更新时间，须人工审查频率、抖动和多轮实测。** 没有自动解锁、非零推力或 flight_started，也未证明现场旧日志就是这个原因。 |
| **C2 / P2 IPC 单轮工作/回复积压** | 自有临时 Unix socket，单轮不超过 4096 字节的多条 STATUS 会同步执行全部 handler/状态编码，已有 64 KiB 输出门到下一 poll 才检查。原代理保存了 350 次回调和约 1.43 MB 合成回复；实际 SystemNode 状态编码＋假 ROS 的对应保存记录为约 **4.11 MB、x86_64 单次 11.9 ms**。生产 tick 的 cycle/final 门仍会拒绝过期输出。 | `candidates/ipc_send_budget/CANDIDATE_NOT_DEPLOYED.patch` 在每条回复后立即执行原背压策略。独立审查发现初候选把 drop 放进业务 catch，会在 disconnect 自身抛错时重复通知；已只修隔离候选，精确 **2 条请求**验证旧候选通知 2 次、修候选 1 次且原错可见。**最终候选未重跑突发**，三类补丁均 apply-check 通过。该策略不限制单个 handler 分配，也不限制很小回复的总工作量。早断连涉及原下降/准备取消和 LAND/STOP 排序，需人工审核；不能将主机记录外推成实机 CONTROL_LOOP_STALL 或动力越权。 |
| **C3 / P2 start 退出竞态** | `native/tools/start.py` 中 poll 判活动后，节点自然退出，killpg 抛 ProcessLookupError，可跳过本次自建 Agent 的后续回收。只用假进程/假信号三场景复现，没有发真实信号。此工具不是当前正式导航服务的运行入口。 | `candidates/start_exit_race.patch` 只在 ESRCH 后再次确认节点已退出才继续原清理；仍活动/未知则保留原错和 Agent，外部 Agent 不回收。三场景原/候选轨迹保留。涉及控制器/传输生命周期，**不应用到生产，需人工审核**。不宣称已证明实机持久 UART 占用或电机异常。 |
| **C4 / P2 standby 启动失败后 join** | 真实 Thread 对象，注入启动前失败：AgentTransport.close 对未启动线程 join，次生 RuntimeError 可中断后续清理和监督器重试；原启动错仍作为 context 可见。没有启动真实线程/Agent。 | **不提供简单 skip-join 补丁**。`ident is None` 或 `not is_alive()` 不证明线程未来不会运行：CPython 启动底层线程后还等待 started，异步异常可能发生在两者之间。盲目清成员可能丢失已获准运行的恢复者。先明确启动/关闭发布握手和资源归属；理由、CPython 行号/SHA 已保存。 |

候选都在 [evidence.zip](evidence.zip) 的 `candidates/` 下，生产源码未修改；[候选索引](candidate_index.json) 给出版本、SHA 和原/候选复现。C2 的旧 patch、旧 SHA 和旧运行仍保留，不能把旧突发结果说成最终 patch 的新验收结果。

## 历史故障：缺陷与合理拒绝分开

| 项目 | 本轮读码与验证结论 |
|---|---|
| `CONTROL_LOOP_STALL` | Core 检查实际相邻 step elapsed，Node cycle 预算涵盖回调后的 IPC/Runtime/发布上下文；单线程不能抢占正在运行的慢回调，逾期拒绝是安全保护。C2 揭示一种工作积压来源，但没有复现实机 stall。保留保护，下一步采集板上各段耗时。 |
| `VERTICAL_VELOCITY_INCONSISTENT` | 目前仅残留在历史可恢复原因中；速度与 z_deriv 差是诊断 warning，不独立停机。非法数据、样本回退/缺口、位置变化与导数积分不符仍有独立检查。原 guard 回归通过，本轮未删除保护。 |
| `PARTIAL_OR_FAILED_SETPOINT_PUBLICATION` | 目标和心跳是非原子动作；目标已返回而后续心跳失败不能算完整成功。相关 **522 组**发布组合及离线时序回归通过；本地 sink 返回不是 PX4 已采用的证据。 |
| `HEARTBEAT_FINAL_CONTEXT_CHANGED` | 最终每阶段重新取当前 context，TTL/deadline 跨界必须拒绝；未解锁准备只有既有窄延期，不能据此放宽实际动力发布。原时序恢复测试通过。 |
| 地面高度参考 | C1 属于时间证据问题。原未解锁地面可跟随缓慢一致漂移；armed/arm 意图后锚点冻结，reset/跳变和包络门保留。本轮没有允许空中重设零点。 |

另外纯配置反例发现 **P3 错误合同缺口**：超大正负 JSON 高度整数、超大 probe 单调戳会抛 OverflowError，NaN/Inf/负值/bool 仍被拒绝。没有接受坏高度或授予地面许可；这些门涉及原服务准入，生产保持原样，记录为后续隔离错误提示修复，不混同为飞行失控。

并发/积压检查也逐项读了实际所有权：默认 ROS 回调和 tick 使用单线程；离散事件队列满后保留溢出，健康尾帧不直接清除。AsyncLog 的 pending/队列/字节计数由同一 mutex 保护，截止时间与丢弃计数为 atomic，消费者释放队列锁后才 write/poll。它仍可能等待互斥量或分配器，析构 join 没有硬实时上界；标准流缓冲替换要求日志寿命包住全部写入者。没有据此宣布任意外部线程/任意 OOM 无竞争；TSan 尚未取得有效覆盖。

## 真实验证结果与失败记录

| 验证 | 实际结果与范围 |
|---|---|
| 最终根 Python 全量 | **635 方法：632 通过、3 ERROR；0 FAIL、0 跳过**。新增 23 方法；含 82 个当前主机 C++ 测试程序聚合、控制差分、生命周期和资源故障注入。最终耗时约 89.8 s。 |
| 第一轮全量 | 635 方法，1 FAIL＋3 ERROR，原日志保留。FAIL 是 Stage4 整文件 AST 守卫正确发现本轮日志功能修复；现完整 deploy 锁定独立 `6c4cc7d`，类外编排和 `record/step` 仍对旧基线逐项相等，原安装器 AST/构建轨迹断言均保留。独立复核 8/8 后才重跑全量。 |
| ASan/UBSan | 新 IPC 合同 **122 条断言**、C1 候选四轨迹通过。初版新 IPC 测试的内嵌 NUL 夹具误传长度，ASan 报测试自身越界；修正夹具长度后 strict/sanitizer 重跑通过。原失败与源码快照保留，**不是生产内存缺陷**。C2 初候选 sanitizer 证据是旧 SHA，最终版本范围另记。 |
| 真实 Humble 采集生命周期 | 隔离 localhost 域 222，先后核验空图，真实被动采集器 **3 次正常创建/订阅/销毁/关闭 Context**，无消息时报告9个topic，全部 Context 关闭。只创建观察订阅和 ROS 元数据，无合成 `/fmu` 发布者、生产控制器、Agent 或设备。异常路径由假资源注入验证，不冒称真实磁盘/ROS 故障。 |
| Clang 静态分析 | 3 个实际控制测试单元（状态不变量、发布组合、RC）exit 0、无诊断；另有 C1 驱动分析。不是整 ROS 依赖树或全路径证明。 |
| cppcheck 2.7 | **未取得有效通过**：`attitude_ros_parameters.hpp:42` 内部 AST 错误；工具 exit 0 也不能称成功。XML/命令保留。 |
| TSan | 最小无竞态线程/atomic 探针编译成功，运行 **exit 66**：`FATAL: ThreadSanitizer: unexpected memory mapping`。尚无有效 FC01 TSan 覆盖。 |
| 静态/格式与原行为保护 | **128 份 Python** 编译语法、变动 Python Black、新 C++ clang-format、diff 检查通过；**97 份受保护源/配置/发布包文件原字节相同**。三份候选 git apply --check 通过，未应用。 |

三项 ERROR 仍为既有 `Curated source payload exceeds extractor bounds`：两个 navigation_install.BundleTests 和一个 vertical_diagnostic_bundle 精确载荷测试。[Stage 4](../python_stage4/README_zh.md) 已在当时不可变起点独立复现；本轮没有调大预算、删测试或伪造新包身份。**发行仍阻塞，没有产出本轮新安装包。**

本轮 C++ 生产文件、MSG、CMake、YAML 全与起点一致，因此没有为两个 Python 清理修复重复 Humble 全构建；不受影响的 native Python 41 项、ARM64、相机/SLAM 性能未重跑。不把前阶段结果当成本轮新结果。视觉仍关闭、VelocityCommand 仍只读；原人工 RC、1.5 m 起飞、推力参数、接管、Kill、Failsafe 和最终发布行为未改。

IPC 并行代理曾被平台风险检测中止，原始本机测试证据保留；后续由另一审查者只读复核记录，未扩大或重复请求突发，仅验证候选断连异常边界。没有将未完成代理的部分结果冒称一次完整独立审查。

## 验收边界与下一阶段

可以继续最终离线验收与无动力 ARM64 构建准备；**不能据此认定可启用视觉、可自动套用候选或已有合格新版安装包**。C1 是明确的准入证据缺陷，应在扩大实机测试前优先人工审核；C2 需要板上工作量/循环段耗时记录；C3/C4 需先完成退出责任和线程握手评审。有限输入通过不是数学安全证明。

仍需目标板 CPU/内存/延迟、land实际频率/抖动、PX4源时钟与接收时间、真实状态/ULog采用记录；真实视觉的标定、尺度、时钟、质量与reset合同仍缺。既有 Stage5 低质量 API 历史候选保持原结论，本轮没有重复研究或将其提升为生产缺陷。

逐文件范围和未深入部分见 [review_ledger.json](review_ledger.json)，具体修改理由和 SHA 见 [changed_files.json](changed_files.json)，命令/计数/引用保护见 [verification.json](verification.json)。[evidence_index.json](evidence_index.json) 对证据包每成员给出长度/SHA；归档排除二进制、构建/安装缓存，保留原失败、最终结果、候选及独立交叉审查。

本轮完成本地修复、测试、审查和报告后结束，供队列第 7 项基于最终实际 HEAD 接续。未给出的最终验收任务不由本阶段自行扩展为部署或动力测试。
