# FC01 离线控制运行稳定性：源码审计与最小修复

日期：2026-11-07。工作分支：`dev/offline-control-stability-20261107`。本轮范围为伴随计算机控制软件；不部署、不改飞控参数、不刷固件、不执行解锁或动力试验，不重复 UART 收发与 DDS 基础连通性测试。

## 1. 修复前审计结论

有两处能够直接由源码确认的时序问题，也有尚不能确定原因的现场现象。以下结论先于修复验收，不能用构建成功代替实机结论。

| 项目 | 已取得的证据 | 结论与处理范围 |
| --- | --- | --- |
| 发布边界的计时点过早 | 基线 `boundary_context()` 在 `snapshot()` 之前采样 `steady()`，随后使用这个旧时间计算控制有效性；`snapshot()` 内含多次 DDS graph 查询 | 能确认 snapshot 自身耗时未计入当次边界预算。修复应在 snapshot 完成后重新采样单调时间；保留最终安全复核 |
| 当前轮回调未计入发布预算 | 基线在 `spin_some()` 之后才由 `tick()` 设置 `control_started_`；上一轮诊断不能及时阻断本轮首次长回调 | 能确认当前回调耗时存在检查盲区。预算应从回调执行前开始，覆盖回调、控制和发布前检查 |
| 每轮临时 executor | 基线主循环调用便利函数 `rclcpp::spin_some(node)`；该便利路径创建临时 executor，并添加/移除节点 | 能确认存在可避免的生命周期工作。改为单个持久的单线程 executor，保持回调后执行 tick 的顺序。不能据此断言它造成了 336 ms 停顿 |
| 多次 graph 查询 | 基线每次 snapshot 都查询所有权、反馈发布者和控制订阅者；发布边界又多次获取 snapshot | 能确认重复查询存在，尚无现场分段耗时证明它是瓶颈。本轮先集中查询代码并记录次数和耗时，每次 snapshot 仍实时查询，不引入拓扑缓存 |
| 源时间年龄约 0.489 s | 已有报告中 `local_age_s≈0.000018`，但 `local_source_age_s≈0.488878`、`local_sample_age_s≈0.49072`；姿态源年龄约 0.48392 s，阈值为 0.5 s | 接收新鲜与源样本新鲜是不同证据。真实源延迟、DDS 时间映射偏移、回调排队和主机时钟变化均尚未被排除；不能认定 NTP 或 graph 是根因，也不能扩大阈值 |
| 历史 336 ms 控制周期 | 报告出现 `max_cycle_period_s≈0.336`，现行阈值为 0.3 s | 确认曾记录过超限周期，不能从原有总量指标分辨回调、IPC、graph、日志或调度耗时。需要本轮增加的分段证据 |
| 严重故障锁存 | `boundary_fault_latched_` 阻止重新准备；源戳倒退由 `Stamp`/样本进展跟踪永久记忆；部分发布错误进入 SAFETY | 这些保护不能普遍自动清除。只研究尚未出现解锁意图、从未发生部分发布的地面准备阶段暂时失鲜恢复 |

现场尚未确认：修改后 ARM 平台实际周期上界、源时间差的成因、PX4 采用设定值、真实离地或自主飞行。上述内容仍为“待实机确认”。

## 2. 安装包、源码与当前工作区的对应关系

本地存在 `dist/FC01_takeoff_readiness_20261103_source.tar.gz`，其 SHA256 为：

```text
0b6a246ee3c557c79822ad1304fe56ea7368743d192e5f6a42f61fe5b56141bf
```

该值与 `dist/FC01_takeoff_20261103_RELEASE.json` 一致。发布清单记录基线提交为 `6c5b185526b7ac194138d05ec9698fccadf288c0`，分支为 `fix/takeoff-readiness-20261103`。修复开发前工作区 HEAD 为 `fcc35162536da75a00e0ae995e8ef50cf2244d96`；这段提交差异为 UART 审计工具、测试与文档，未修改本轮涉及的原生时间校验、控制运行时、发布边界、配置生成和 UI 行为。

逐文件比对本地源代码归档与修复前工作区：控制源码与归档一致；差异限于 README、既有通信/起飞审计文档和用户在 `fc01_config_check.cpp` 中的缩进/空白编辑。用户编辑应保留，不作为本轮修复提交内容。

发布清单记录安装器 `FC01_OrangePi_Navigation_20261103_takeoff.sh` 的 SHA256 为 `0b42777100ad5f727f3c9fba1b1d7c272c9bc21790fda3344c33668b6cc87408`，与用户交接材料一致，但本地没有该安装器原文件。因而本轮能够独立核对的是源代码归档和发布清单，不能独立证明香橙派当前二进制、安装器内所有文件和 `runtime.yaml` 的实际字节一致。用户提供的现场安装信息与交接审计是部署版本关联证据；实际运行文件哈希仍需在需要验收时只读留存。

清单中的“182 项 Python、60 项 C++、x86 ROS 2 Humble 构建通过”是 20261103 的历史结果，不是本轮修改后的测试结果；它也未声称 ARM 构建或飞行验收完成。

## 3. 两条故障链与实际源码入口

两条历史错误必须独立分析：

1. `RC_PRESTREAM` 中出现 `publication_blocked` → `PUBLISHER_BOUNDARY_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE`。含义为该次发布边界未满足源时钟/样本年龄要求，不能只由错误字符串判定到底哪一路数据失效。
2. `PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL`。含义为姿态目标可能已经进入本地发布 sink，但在后续最终复核时控制预算失效，Offboard 心跳被阻断；必须保留部分发布故障处理。

主要调用关系如下，原交接中的行号对应 20261103 包，修改后应按函数名定位：

| 路径/函数 | 作用 |
| --- | --- |
| `source_clock.hpp` 的 `source_stamp_in_clock()`、`source_sample_in_clock()`、`SourceSampleProgress` | 分别校验信封时间、原始样本时间和样本实际推进；新信封不能为冻结样本续命 |
| `runtime.hpp` 的 `Stamp::accept()` | 记录接收单调时间，拒绝不前进或倒退的信封，保留倒退证据 |
| `fc01_system_node.cpp` 的订阅回调、`snapshot()` | 接收反馈，计算接收/源时间有效性、RC 状态及端点所有权 |
| `Runtime::step()`、`Core`、`RcNavigationStart` | 飞行阶段与准备状态机、控制计算、RC 权限判断 |
| `SystemNode::boundary_context()` | 发布前重新获取反馈与唯一控制源证据，校验本轮执行预算 |
| `publication_boundary.hpp` 的 `publish_attitude_resampled()` | 三次 context：初次准入、打包后目标发布前、目标发布后心跳发布前 |
| `SystemNode::refuse_publication()`、异常捕获 | 明确记录拒绝、阻断输出；部分发布及严重故障保持锁存 |
| `Runtime::reprepare_rc()` | 仅在既有安全恢复条件允许时重新准备，不绕过 publication fault |
| `main()` | 单线程执行订阅回调、tick、周期观测与剩余周期等待 |

`source_clock_validated=true` 表示程序完成了校验，不能当作 PX4 timesync 已收敛的独立证据。三次边界复核均须保留；允许发布只证明本地执行路径通过，不能证明 PX4 收到或采用。

UI 在当前反馈失鲜时保留 WAITING，用于避免把旧进程的 SAFETY 当成新进程当前状态。已有单元测试明确约束这一行为，本轮不改变它。

## 4. 完全离线的时间约定

本机 TTL、周期、等待窗口和耗时使用 `steady_clock`。跨机源样本年龄仍使用 ROS/DDS 所约定的主机时间域与 PX4 uXRCE-DDS 的时间映射；不能直接把 PX4 原始启动时间与 Linux `steady_clock` 相减。稳定但日历日期不准确的 Linux 时钟不会因其日期本身失效；是否可用取决于跨机映射和源样本年龄，而非 Wi-Fi 或 NTP 状态。

本轮不要求联网、不主动校时、不关闭源校验、不凭接收时间替换源时间、不向源戳添加估算偏移。保留 0.5 s 源年龄约束、现有未来时间容差和 0.3 s 控制停顿阈值。真实失步、PX4 重置、源戳倒退或控制停顿仍应拒绝输出。

新增主机时钟诊断比较“系统时钟增量”与“单调时钟增量”，不相减两者绝对 epoch。其作用是记录跳变线索，不是补偿数据年龄或授予控制权限。缺失/不可用的年龄和中间件接收时间显示为 `null`，不得用零冒充有效测量。

## 5. 最小修复与诊断结构

以下为当前候选修改的审查范围；完成后的构建、回归结果在第 7 节单独记录，不能由实现描述推定验收通过。

| 修改 | 目的与限制 |
| --- | --- |
| 持久的 `SingleThreadedExecutor` | 避免每轮临时 executor；保持单线程回调→控制顺序，不并发共享状态 |
| `CurrentControlCycle` | 在回调前开始预算；snapshot、控制和最终发布检查使用新采样单调时间；本轮过期不能靠再次调用校验恢复 |
| snapshot 先查询 graph，再采样接收/源年龄 | 防止 graph 查询耗时被旧计时点掩盖；没有拓扑缓存，每次发布复核仍检查所有权 |
| 六路源诊断 | status、land、local、attitude、flags、battery 分别记录信封/样本时间、接收年龄、源年龄、拒绝原因和倒退证据 |
| 回调接收记录 | 对比回调时的源年龄与发布时的源年龄；RMW 接收时间可用时额外记录到回调的间隔，不可用时为 `null`。RMW 时间来源质量未经确认前不能单凭该值宣布网络或排队根因 |
| 分段耗时 | 记录 callbacks、IPC、runtime、snapshot、graph、boundary、reporting，及 snapshot/graph/context 次数；发布对中的三次 context 各自计时 |
| 拒绝与超限日志 | 拒绝时记录六源证据、当前轮本地 target/heartbeat 执行情况；超限提示限制频率，避免每 tick 大量写盘 |

snapshot、graph、boundary 是包含关系，诊断耗时存在重叠，不能把这些字段直接相加当成完整周期。`last_completed_cycle_work` 是上一完整周期；故障当轮应看故障记录中的 `cycle_work` 和 `pair_context_seconds`。分段计时能缩小定位范围，但仍不构成调度延迟的完整 profiler。

本轮不引入 graph guard-condition 缓存；新增端点或竞争控制源仍由每次 fresh graph 查询拒绝。因而不存在“缓存已验证、忽略后续 graph 变化”的新许可窗口，也不承诺消除全部 graph 查询开销。

## 6. 暂时失鲜与严重锁存的边界

严重故障处理保持不变：已发生部分目标/心跳发布、时钟/样本倒退、解锁或控制权不确定、已进入实际飞行流程，以及既有 publication fault，均不能由暂时失鲜恢复路径清除。

允许进一步实现和验证的窄恢复路径为：仅在 `RC_PRESTREAM`/等待手动解锁的地面准备阶段、尚未出现任何解锁/导航意图、尚无解锁或 Offboard 命令记录、该轮两个发布 sink 都未执行时，对能够明确归类为 local/attitude 年龄到期的情况撤回本轮准备。其余状态、land、failsafe、RC、battery、所有权、地面静止、重置计数和基准一致性仍须有效。不能把源时钟跳变、RC 失鲜或单纯“当前 arm 开关为 OFF”视为足够证据。

撤回准备不刷新源戳、不清空倒退记录、不清除严重故障、不允许继续发送目标；条件恢复后重新经过现有准备约束。重复 arm ON→OFF 的历史不能被最后一帧 OFF 掩盖。具体适配层分类与回归结果以最终代码及第 7 节为准。

## 7. 验证记录

修复前已在本地完成以下离线检查：

- 11 个相关原生 C++ 测试分别编译运行通过：`test_r62c_review_clock`、`test_authority_boundary`、`test_m1_safety`、`test_socket`、`test_rc_reprepare_admission`、`test_control_diagnostics`、`test_publication_boundary`、`test_publication_runtime`、`test_r6_publication_audit`、`test_socket_boundary`、`test_r62c_review_diagnostics`。使用 C++17、`-Wall -Wextra -Werror -pedantic -pthread`，产物位于临时目录。
- `PYTHONPATH=tests PYTHONDONTWRITEBYTECODE=1 python3 -m unittest test_navigation_service -v`：17 项通过，包含当前反馈失鲜时 WAITING 不被历史 SAFETY 覆盖的约束。

原测试已覆盖部分发布、RC 恢复拒绝、源戳倒退等行为，但不能替代这次新增的精确时序回归。候选新增 `test_offline_timing.cpp`、`test_rc_preparation_defer.cpp` 和现有控制诊断测试扩展，重点覆盖：

- 不同稳定日历 epoch 下相同源年龄、真实旧样本和冻结样本不可续命。
- 0.49 s 在最后一次边界变为 0.51 s：已发布目标后必须阻断心跳并保留部分发布故障。
- 主机时钟前跳、后退、单调时间异常、PX4 样本重置及进程重建。
- 本轮回调或 snapshot 使预算超过 0.3 s，包括 0.336 s 情形；发布前和发布后超限分别处理。
- 无解锁意图的地面准备暂时失鲜撤回；解锁意图、RC 失鲜、重置、竞争源、部分发布及严重锁存均不得借此恢复。

修改后离线回归、真实 ROS 2 Humble 构建及接口编译检查结果：**待本轮执行完成后补录**。ARM 运行、真实 PX4 采用和飞行均未执行。

## 8. 一次有界拆桨地面验收

此步骤供用户后续手动安排，不表示本轮已经授权部署或启动飞行任务。

1. 拆桨、保持未解锁，确认实际运行配置 `arm_enabled=false`、独立遥控解锁策略和唯一控制进程。保留现有 DDS V3、原 DDS Agent 与第二 UART 配置；不用重复 CRC 接收或香橙派回环测试。
2. 通过现有人工服务流程选择待验收二进制，留存运行文件哈希、实际配置、进程 PID 和启动时间。若这些证据不匹配开发提交，先停止本次软件验收；服务切换不由本轮自动执行。
3. 在无 Wi-Fi/NTP 的既有离线条件下观察一次 30–60 s。保持安全遥控状态，不发起解锁、起飞、导航或追击任务。保存状态与 stderr，检查六路 `source_timing`、主机时钟增量事件、`last_completed_cycle_work`、graph 查询次数和真实端点计数。若自然出现准备拒绝，保存当次 `publication_blocked` 及三次 context 耗时；未触发发布路径应如实标为该路径未实机验收。
4. RC 失鲜/接管只由现场人员在拆桨安全状态下按原有操作输入核验，不能由自动脚本制造解锁或飞行状态。部分发布、真实时钟跳变及 336 ms 停顿的强制注入仅在离线 mock/回放测试中进行。唯一控制源变化的拒绝由离线回归验证；现场真实发现竞争源时，应拒绝控制并保留证据，不为测试自动启动第二控制节点。
5. 区分结论：构建通过、进程运行、反馈有效、发布边界通过、PX4 采用、真实起飞是不同验收项。此次观察最多证明地面软件路径，不宣称自主飞行能力。

## 9. 根因仍未确定时的最小取证

只采上述一次观察中的软件诊断，不重新开展全套串口测试：

| 待区分原因 | 最小证据与解释范围 |
| --- | --- |
| 源样本到达前已老 vs 回调后被拖慢 | 同一路 `callback_envelope_age_s`/`callback_sample_age_s` 与拒绝时源年龄、接收年龄；到回调时已老不能单独证明网络延迟，也可能是时间映射偏移 |
| 回调排队 | 可用的 `rmw_to_callback_s`、callbacks 总耗时；RMW 字段不可用或时间域未确认时保持未知 |
| 主机时钟跳变 | `host_clock_*` 事件、系统与单调增量差；不能用错误日历日期直接解释年龄 |
| snapshot/graph/IPC/报告耗时 | 同一故障轮 `cycle_work`、三次 context 耗时与上一完整轮记录；注意包含关系和调度空白 |
| 同步偏移 vs 真实传输/生产延迟 | 若所有源在回调入口已呈稳定共同偏移，仍需与当时 PX4→主机时间映射证据交叉核对；现有 connected/timesync 日志仅是背景，不足以单独排除偏移 |

若没有出现原故障，只能报告本次有界观察未复现，不能宣称 .489 s 或 336 ms 的历史根因已消除。

## 10. 合并与回退范围

先独立记录本审计报告，再将运行时修复、诊断和直接相关回归提交到当前开发分支；最终提交哈希与修改文件清单由交付记录补齐。保持 UART 拓扑审计提交历史，不重写或回退已有串口配置。不纳入用户 `fc01_config_check.cpp` 的格式编辑。

合并到主线时逐项核对：`source_clock.hpp` 的阈值和判断保持、`publication_boundary.hpp` 三次复核保持、PID/推力/保护阈值保持、DDS V3 与 UART 配置保持、严重故障与 RC 接管测试通过。需要回退时仅回退本轮软件修复提交，不通过改参数或清故障伪造恢复；香橙派服务版本回退由用户另行安排。本轮没有运行现场服务切换、参数修改或固件刷写。
