# FC01 高频反馈积压：ARM64 报告复核与有界回调处理

日期：2026-11-07。前序基线：`97eb691`，即离线控制时序修复及交付报告完成后的版本。本轮仅处理导航控制器对已收到反馈的消费能力；不部署、不重启 Agent、不修改飞控参数或固件、不操作动力。

## 1. 审计结论

新 ARM64 进程已经连续进入 `RC_READY_WAIT_ARM`，但姿态和位置反馈在回调开始时的年龄持续增加。因此，单次显示 READY 还不足以通过稳定性验收。这次报告提供了支持“应用消费高频历史样本”的直接证据；不应继续把所有源年龄问题笼统归因于离线日历或 NTP。

源码每个控制周期只调用一次 `SingleThreadedExecutor::spin_some()`。Humble 的这条执行路径对本次收集到的每个就绪 subscription 执行一次；它不保证一次调用排空该 subscription 的 DDS 历史。配合 `SensorDataQoS().keep_last(10)`，应用可以每轮正常收到反馈，却一直落后于最新样本。相对本轮开始重新计算的 `now()+period` 又允许小幅调度迟到逐轮积累。需要改善反馈消费能力，同时保留源时间校验、单线程顺序、控制预算和三次发布边界检查。

实现依据为实际工作树与 `97eb691` 的差异，未直接套用旧安装包行号。实机进程路径确认部署目录为 `20261107_stability`，但报告没有运行二进制 SHA256，目录名称不能证明与本地提交逐字节一致。旧 20261103 审计仅作为调查线索。

已复现的是应用消费上限导致积压的机制。现场初始传输延迟、实际 RMW epoch 和 PX4/Agent 时间映射的剩余误差，不能从现有报告唯一分解。独立被动订阅者用于下一轮对照：只有控制器增长时继续查消费；两者同时增长时再查传输或跨机时间映射。不把“持续积压已修复”扩大为“所有延迟来源已排除”。相关 Humble 实现依据：[executor](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/executor.cpp#L416)、[ready subscription 选择](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/include/rclcpp/strategies/allocator_memory_strategy.hpp#L246)。

本报告的现场数据没有发生 336 ms 停顿，也没有出现主机时钟跳变事件。因此本轮不能解释此前每一次 FAULT，更不能宣称此前的 336 ms 根因已经查清。旧进程的 FAULT 与新进程的 READY 必须分别统计。

## 2. 证据来源和进程分段

直接读取的原始报告：

```text
/media/jikl62/KINGSTON/FC01_navigation_report_20261021_132605_440106.zip
SHA256: 8d259e6f30784f5669e44530a1666aef71f48884d261256e77fedadfbd3e00c9
大小: 524730 bytes
```

ZIP 共 11 个文件，安全解包到临时目录后分析；未修改 U 盘原件。`controller.log` 由日志泵分块写入，同一 JSON 可能被拆成数行，需先去除每块日志前缀、重组 JSON，再按 `controller_pid` 分段。三份 controller 日志重组得到 2144 条完整记录，无 JSON 解析失败；其中新进程有 250 条 state。

| 对象 | 报告中的直接证据 |
| --- | --- |
| 新 supervisor | PID `4416`，`service.txt:4`、`status.json` |
| 新控制器 | PID `4433`，来自 `20261107_stability/install/lib/.../fc01_system_node`，`service.txt:11` |
| 已有 Agent | PID `1870`，`serial -D /dev/ttyS2 -b 115200 -v 0`，`service.txt:9` |
| Agent 归属 | `agent_owned=false`，本次借用已有进程 |
| 旧故障 | `journal.txt:8–19` 属于 supervisor `754`，发生于新服务启动前 |
| 新启动 | `journal.txt:33–38`，13:24:00 启动，13:24:03 PRESTREAM，13:24:04 READY |

用户粘贴的终端记录与 ZIP 中的 service、journal、status 相互吻合。记录中的旧系统日期不能单独解释源年龄；此处使用控制器单调时间分析增长趋势。

## 3. 新 ARM64 进程的量化结果

PID `4433` 的 state 覆盖单调时间 `426.453377 → 551.319560 s`，共 `124.866183 s`。阶段分布为 5 条 IDLE、3 条 RC_PRESTREAM、242 条 RC_READY_WAIT_ARM。该进程样本中未见 publication fault 锁存、已解锁或非零推力，末次 target/heartbeat 本地提交分别为 `2441/2441`。这些提交不证明 PX4 采用，更不证明离地。

| 指标 | 本次 ARM64 结果 |
| --- | --- |
| 记录到的上轮周期 | 均值 `50.1443 ms`；进程累计最大 `50.6065 ms` |
| stall 计数 | `0` |
| 回调处理时间 | 均值 `0.434 ms`，记录样本最大 `0.765 ms` |
| 控制处理时间 | 均值 `1.816 ms`，记录样本最大 `3.444 ms` |
| graph 查询 | 正常预发送周期 4 次 snapshot、84 次查询；合计均值 `1.398 ms`、记录样本最大 `2.760 ms` |
| 主机时钟事件 | `host_clock_event_count=0`；记录的 wall/steady anchor 差约 `−1.63…+0.96 µs` |
| 位置回调 RMW 时间差 | 开始 `21.3 ms`，结束 `438.1 ms` |
| 姿态回调 RMW 时间差 | 开始 `43.4 ms`，结束 `453.4 ms` |
| 其他四源 RMW 时间差 | 均值约 `25–26 ms`，记录样本最大 `50.6 ms`，无同类线性增长 |

上述耗时是状态采样能观测到的统计，不应把每个字段的样本最大值当作全程精确最大值；`max_cycle_period_s` 本身记录了全程累计最大周期。

稳定段单调时间 `431.473507 → 551.319560 s` 中，2390 对提交耗时 `119.846053 s`，有效控制提交频率为 `19.94225 Hz`。对应位置源戳只推进 `119.511912 s`，姿态源戳推进 `119.505856 s`，分别落后额外 `334.1/340.2 ms`。每次提交对应的源戳推进约 `50.005/50.002 ms`，符合每轮消费一个约 20 Hz 历史样本的模式。

对该稳定段 240 条状态进行线性拟合：

| 诊断量 | 增长速度 | R² |
| --- | --- | --- |
| 位置回调源年龄 | `2.727 ms/s` | `0.9923` |
| 位置 RMW→回调时间差 | `2.716 ms/s` | `0.9917` |
| 姿态回调源年龄 | `2.726 ms/s` | `0.9936` |
| 姿态 RMW→回调时间差 | `2.727 ms/s` | `0.9918` |

其他四源的回调源年龄、RMW 时间差没有这种共同增长；主机日历增量与单调时间增量拟合斜率为 `1.00000000257`。这与高频 subscription 的历史排队相符，明显削弱“全局主机时间漂移”对本段现象的解释。

`rmw_to_callback_s` 是本机 wall 时间与 RMW `received_timestamp` 的差，只有确认 RMW 接收时间使用同一 host epoch 时才能解释为队龄。报告的构建日志显示默认 `rmw_fastrtps_cpp 6.2.10`；报告没有保存运行进程的 `RMW_IMPLEMENTATION` 环境及完整动态库身份，不能仅凭构建默认值证明现场实际选择。因此还应结合源戳、单调时间、其他源对照和本地真实 ROS 复现，而非单独依赖这一字段。

计数字段也应分开解释：`callbacks_total` 是收到并执行的回调数，`callbacks_this_cycle` 是当前周期回调数，`accepted_total` 及 `last_callback_accepted` 仅表示信封时间戳被接收入口接受。它们不表示原始 sample 有效、数据仍新鲜、权限已满足或控制器允许输出；必须另外检查 sample 推进、`source_reason`、`receipt_reason`、各 fresh 字段和最终发布判定。新信封携带旧测量仍可能增加 accepted 计数，并继续被控制侧拒绝。

## 4. 实际配置及保留约束

`runtime.yaml:3–21` 确认：`use_sim_time=false`、`flight_enabled=false`、`arm_enabled=false`、控制 20 Hz、姿态和位置源年龄限制 0.5 s、stall 0.3 s。`flight_actions_enabled=true`、`takeoff_enabled=true` 是现场导航待命配置，不能将它描述为所有动作门均关闭。`rc_target_height=1.5`，视觉关闭。

反馈 QoS 在源码中为 `SensorDataQoS().keep_last(10)`，用于姿态、位置、状态、落地、电池、flags、RC 和 ACK。这里的 10 是 DDS 历史深度，不是视觉频率或 PX4 串口频率。本轮保持该 QoS 和所有安全阈值，避免通过丢弃所有旧记录掩盖重排、重置或安全输入。

三次发布 context 复核、graph 唯一来源查询、RC 接管、原始样本时间推进、严重时间倒退锁存均继续保留。多次收集回调的时间属于同一个当前控制周期预算；不能额外获得一个新的 0.3 s 预算。

`arm_enabled=false` 表示禁止程序解锁；`flight_enabled=false` 是另一条通用控制入口的开关。这两项均为 false，仍可能是允许 RC 独立手动解锁的正常姿态导航路线。是否仅作诊断，必须读取 native state 中真正的 `flight_actions_enabled` 和 `takeoff_enabled`，不能由 supervisor 的角色元数据推断。

## 5. 修复范围与验证状态

最终候选已经包含连续异常往返样本保护，并完成主回归、真实 Humble 构建和生产 node 合成复测。下表与日志注明各自覆盖范围；所有 ROS 实验均为本机隔离的合成验证，ARM64 实机验收仍待完成。

修复方案是在同一单线程 executor 上进行有界的多次就绪回调收集，让控制 tick 使用已收到的新反馈。20 Hz 时，决定是否开始下一次收集的预算为周期的十分之一，即 5 ms，同时限制最多 10 次收集；出现需要处理的离散安全事件时停止额外收集，进入 tick。一次完整 `spin_some()` 不能抢占，其中的回调可能超出 5 ms，因此这是协作式收集预算，并非硬实时耗时保证。当前完整控制周期及发布边界仍独立检查实际超时。

这一处理不把接收时间改成测量时间，不给旧样本重新盖章，也不放宽 0.5 s。若输入实际一直过期、时间倒退、控制确实停顿，原有拒绝及锁存仍应发生。

| 处理层 | 本轮实现与边界 |
| --- | --- |
| 接收 | 同一 executor 做完整的就绪回调收集；空收集、事件、5 ms 协作预算或 10 次上限结束追加收集 |
| 缓存 | 保留原 envelope/sample、原接收单调时间和原有推进/倒退锁存；同一线程更新和读取，无新增共享线程 |
| 控制 | 仍按 20 Hz 调用 tick；控制计算使用接收阶段结束时的缓存，实际周期和当前周期工作耗时仍受 0.3 s 检查 |
| 安全与发布 | 三次边界复核及实时 graph 所有权查询保留，事件先交原控制器判断，接收调度不授予权限 |
| 诊断 | 状态新增每源回调数、每周期回调数、收集次数/预算/停止原因；独立只读探针并行记录原戳和年龄趋势 |

本轮没有改成 depth=1，也没有用无界 `spin_all()` 排空队列。普通连续数值可在同一周期被更新的有效样本替代；ACK、状态/权限变化、重置、非法值及安全越界会阻止下一次收集。原有深度和 BestEffort 本身并不保证传输每个安全事件，本轮仅保证新增的追加收集不会进一步掩盖已观测到的关键变化。高负载回调或频繁事件仍可能耗尽预算，应通过计数和年龄观察，不能宣称任意输入频率下都能清空历史。

连续数据也不能一概覆盖：新增只读 `Core::local_requires_control_step()` 对照原控制样本和原地面基准，发现累计跳变、绝对高度/速度越界、准备稳定窗口中断、起飞制动或离地证据撤销时停止追加收集。垂直反馈窗口用原 monitor 的副本作预判，不推进真实监控状态；原 `step()` 决定是否故障。测试覆盖异常后返回正常的样本序列，也确认健康地面、普通高空下降和 Idle 不会无故阻止追加接收。未新增控制线程或跨线程共享反馈。

本次 ARM64 报告中的 graph 合计耗时没有达到控制预算瓶颈，因而保留即时查询，不引入有失效窗口的拓扑缓存。标准输出日志维持已有有界异步队列，IPC 维持既有非阻塞路径；本轮只修复另行证明的日志退出通知问题。另有可选 CSV 路径在控制线程每 0.5 s 写入并 flush，启用时仍可能受磁盘阻塞影响；当前服务生成器明确设 `csv_path=''`，没有证据将其列为本次现场积压原因，本轮不扩大修改。DDS 断流后缓存不会被重新赋予新时间：源 TTL、接收 TTL、原始 sample 推进仍独立到期；时间倒退、估计器重置及主机时钟跳变仍走既有保护和人工恢复条件，不自动清空严重锁存。

本机真实 ROS 2 Humble 实验采用 localhost 独立 Domain 188、BestEffort/KeepLast(10)、合成数据，未打开串口、未启动 Agent 或控制节点。版本为 rclcpp 16.0.19、rmw_fastrtps_cpp 6.2.10、Fast DDS 2.6.12。

| 25 Hz 输入 / 20 Hz 控制调度 | 首→尾源年龄 | 消费频率 |
| --- | --- | --- |
| 原 free executor，一次收集 | `29.159→388.854 ms` | 约 `20.001 Hz` |
| 持久 executor，一次收集 | `49.362→369.304 ms` | 约 `20.001 Hz` |
| 持久 executor，有界多次收集 | `1.029→30.971 ms` | 约 `24.750 Hz` |

这组实验约 4 s，消费频率包含起止边界误差，结果保存在 `.build/ros_receive_ab/final_results.json`。源年龄包含采样相位，不要求首尾数值相同。原路径到 KeepLast(10) 满后会丢弃更早记录，年龄可能进入平台；不能把平台当成恢复。仅换成持久 executor 仍然积压，说明 executor 生命周期不是充分修复。

独立验证保留了“新信封不能掩盖旧测量”：50 条新信封均通过源时间校验，但对应原始 sample 已老 600 ms，50 条全部被同一生产函数拒绝。12 ms 的单回调实验超过 5 ms 协作预算后不再追加收集，41 次均停在预算边界，积压仍存在；这也验证方案没有硬实时承诺。以上证明处理机制，不证明 ARM64、飞控或实际飞行已通过。

真实生产 node 的对照使用 localhost 独立 Domain 187、40 Hz 合成输入、约 10 s 连续观察。旧基线二进制每轮最多消费一个高频回调，位置 RMW→回调差 `105.595→231.648 ms`、峰值 `273.392 ms`，触发实验的积压断言；这是预期复现失败，不能记作旧版通过。最终候选结果：

| 指标 | 位置 | 姿态 |
| --- | --- | --- |
| 首→尾 RMW→回调差 | `9.290→24.383 ms` | `9.280→24.345 ms` |
| RMW→回调差峰值 | `25.967 ms` | `25.927 ms` |
| 首尾窗口均值之差 | `0.686 ms` | `0.683 ms` |
| 每周期回调数峰值 | `4` | `4` |
| snapshot 源年龄峰值 | `26.273 ms` | `26.273 ms` |

最终候选最大控制周期 `50.253550 ms`、stall 为 0，合成区间没有旧路径的持续增长。这直接覆盖生产接收路径，仍只是 x86 本机结果。零推力预发送 `12/12` 对 target/heartbeat，0 条 VehicleCommand、0 次非零推力；停止输入后 `0.604326 s` 观测到 SAFETY，进程正常退出。该时间包含轮询延迟，不是超时阈值。三次发布检查均由此零推力合成路径覆盖；第 7 节实机观察不请求预发送。

独立 A/B 的 8 项检查通过；被动探针的 9 项纯逻辑及 Humble 5 s、40 Hz 双话题验证通过，每话题接收 198 次回调，没有控制话题 publisher。

首轮主 Python 回归还暴露 `test_async_log` 退出超过 20 s：AsyncLog 析构修改退出谓词时未使用 consumer 等待的同一 mutex，存在通知丢失窗口，另用确定性条件变量时序复现。无当时堆栈，不能声称它已唯一解释那次超时或历史控制停顿。修复限定为同锁发布退出谓词，并新增 100 次空 log 退出回归；与反馈积压分开提交。

显示层同时修正准备阶段的误导：native 明确关闭飞行动作或起飞时，显示 `DIAGNOSTIC` 及实际动作门，保留原 `state.phase/reason` 作为内部阶段，取消 CH5 起飞提示。未知动作门不提示手动解锁。正常 RC 路线仍允许 `flight_enabled=false`、`arm_enabled=false`，在另外三个实际许可明确为 true 时保持原有提示。FAULT、停止及反馈失鲜的优先级不变。

针对显示与服务的 28 项测试全部通过，包含新增的关闭动作门、未知门值、合法手动解锁路线和历史/故障优先级回归：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=tests python3 -m unittest \
  test_navigation_diagnostics.NavigationDiagnosticsTests \
  test_navigation_service.NavigationServiceTests -v
```

最终验证如下；主 Python 内的 C++ wrapper 已包含 C++ 项，不应把各行相加宣称独立用例总数。

| 验证 | 结果与范围 |
| --- | --- |
| 主 Python discovery | `227/227 PASS`，`60.180 s`；日志 `python-safety-final.log` |
| 主机 C++ | 上行包含 64 个可执行测试，严格 `-Wall -Wextra -Werror -pedantic` 编译；新增事件保护含 8 组场景 |
| 真实 ROS 2 Humble 构建 | 最终安全保护后 colcon `1 package finished [25.0s]`，日志 `build-safety-final.log` |
| 生产 node 40 Hz smoke | PASS，指标见上文，报告 `ros_smoke_safety_final_40hz.json` |
| 独立 executor A/B 工具 | 8 项 PASS；覆盖过载历史消费、旧 sample、慢回调和空收集 |
| 被动年龄探针 | 9 项纯逻辑 PASS；真实 Humble 双话题观察通过，无控制话题 publisher |

主日志位于 `/tmp/fc01-feedback-drain-20261107-tests/`，独立 A/B 与探针结果位于 `.build/ros_receive_ab/`。此前还运行过 `native/scripts/test.sh` 的 3 个独立 repro、adapter stub 和 37 项 native Python，均通过；最后追加的 Core 只读 hint 由上列最终 227 项中的 64 个 C++ 再覆盖。测试包括源戳失鲜/冻结/回退、主机时钟变化、RC 接管、重置、严重锁存及三次边界，不以 READY 作为安全恢复证明。

开发专用 `native/tests/ros_receive_ab/` 不进入离线安装 payload；实验源码保留在 Git，extractor 的 300 文件上限保持原样。安装包从已提交源码生成，保留完整运行时和必要回归，不含本机 x86 二进制或未提交编辑。

## 6. Agent 1870 为何保留

`journal.txt:20–31`、`standby.log:5934–5944` 记录旧控制器退出后，supervisor 因缺少新鲜的未解锁接地退出证据保留自建 Agent 1870。新服务启动后复用它。systemd 的 leftover 提示与这个生命周期设计吻合，不能据此认定 Agent 损坏或必须重启。

| 源码入口 | 当前行为 |
| --- | --- |
| `launcher.py:198–212` | 按同 UID、相同设备、115200 选择已有 Agent；多个匹配 Agent 或额外串口 owner 会拒绝启动 |
| `launcher.py:345–354` | 仅在拥有该 Agent、控制器已退出、1 s 内 IDLE/SAFE_DISARMED、未解锁、接地且状态新鲜时终止它 |
| `standby.py:374–391` | 共用 launcher 锁；仅在没有可复用 Agent 时新建 |
| `standby.py:418–425` | 每秒检查借用进程 cmdline 是否仍相同 |
| `standby.py:453–461` | 只清理本轮自建 Agent；借用的 Agent 保留 |
| `navigation_install.py:50–53` | `KillMode=process`、不强制 SIGKILL，允许保留通信进程 |

已借用进程的 executable hash、启动时间、环境变量和 Domain 没有被当前 `choose_agent()` 核实。`preflight()` 只核实新入口的 Domain 0，`navigation_boot.sh:7` 和服务 unit 只固定新进程环境。报告中没有证明 Agent Domain 不匹配，也没有证明其匹配条件完整；这是诊断元数据缺口，本轮不因此重启或替换 Agent。

当前 service 链路固定 `/dev/ttyS2`、115200，与现场一致。`native/scripts/common.sh` 的 USB/921600 默认值属于另一套 native 入口，不是这个服务实际启动路径，不能混用。`native/scripts/agent.sh` 在该独立入口拒绝已占用串口；本轮不切换入口、不调整第二串口。

没有证据表明新服务启动了第二个 Agent 或第二个控制器。进程检查、共享锁和每次发布前的 ROS graph 唯一控制源检查共同约束这一风险；报告中的 `source_counts=[1,1,1]` 仅是三类反馈来源数量，不能代替完整控制话题所有权检查。

针对已有生命周期/归属逻辑重跑 7 项测试，全部通过：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=tests python3 -m unittest \
  test_launcher.OwnershipTests test_launcher.ReportAndLifecycleTests -v
```

这些测试不打开串口、不运行真实 Agent，也不证明现场 Domain 配置。最小补充取证是只记录 Agent PID/starttime/exe 和与 DDS 有关的环境字段，不导出完整进程环境；本轮消费积压修复不以该取证为前提。

## 7. 一次三分钟拆桨地面验收

本轮不执行候选切换或现场操作。后续由用户安排一次 180 s 观察，前提是拆桨、未解锁，原控制器已由用户按已有安全流程停止，确认无其他 `fc01_system_node`，保留现有 Agent。正常安装/服务默认仍是合法 RC 待飞配置，不是无动作诊断配置；不能宣称安装后不会动作。不要直接编辑 live `runtime.yaml` 再重启服务：`navigation_service` 会重新生成它。裸执行 `navigation.sh` 还会进入安装流程，本节不使用它。

交付的 `.sh` 是带校验的源码安装包，不含预编译 ARM64 控制器。只解包到一个新的独立目录，再利用香橙派已有 ROS、消息 SDK、cmake 和编译器离线构建，不需要切换服务。`--install-only` 必须单独使用，不与运行参数混用；环境变量只为这次解包指定目标目录：

```bash
BUNDLE=/home/orangepi/本次候选源码安装包.sh
NEW="$HOME/FC01_feedback_candidate_20261107"  # 选尚未使用的新目录
FC01_NAVIGATION_INSTALL_DIR="$NEW" bash "$BUNDLE" --install-only
bash "$NEW/navigation_boot.sh" build \
  --prefix "$NEW/install" --build-dir "$NEW/.build/ground-observe"
PREFIX="$NEW/install"
```

已只读核对：安装包在 `--install-only` 分支解包后返回；`navigation_boot.sh build` 仅执行源码 configure/build/install，不调用服务部署，不启动 node 或 Agent，也不下载依赖。这两步可在原服务仍正常运行时完成；只有后面的独立候选启动要求用户先安全停止旧控制器。不要将仅解包后的目录当作已编译完成。

仅诊断候选必须同时关闭下面三个实际门，另外保留 `arm_enabled=false` 和 `flight_enabled=false`：

```yaml
flight_actions_enabled: false
takeoff_enabled: false
rc_navigation_enabled: false
arm_enabled: false
flight_enabled: false
```

最短的只生成配置入口是 `navigation_boot.sh prepare-config --file 新路径`。它不启动服务或 Agent，但生成的基线仍将 actions/takeoff/rc_navigation 设为 true，必须在独立副本中明确关闭；同时关闭 bench 与 visual 路线。以下沿用前面的 `NEW`、`PREFIX`，只写新诊断目录，不改现用服务配置：

```bash
DIAG=$(mktemp -d "$HOME/fc01-feedback-diag.XXXXXX")
bash "$NEW/navigation_boot.sh" prepare-config --file "$DIAG/runtime.yaml"
/usr/bin/python3 - "$DIAG/runtime.yaml" <<'PY'
from pathlib import Path
import sys, yaml
path = Path(sys.argv[1]); cfg = yaml.safe_load(path.read_text())
p = cfg['fc01_system']['ros__parameters']
gates = ('flight_enabled', 'flight_actions_enabled', 'arm_enabled',
         'takeoff_enabled', 'rc_navigation_enabled',
         'bench_arm_only_enabled', 'visual_navigation_enabled')
p.update({name: False for name in gates})
assert p.get('use_sim_time') is False and all(p[name] is False for name in gates)
path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
print({name: p[name] for name in gates})
PY
```

`fc01_config_check.cpp:8` 是专用“手动 RC 导航服务”检查器，强制要求 `rc_navigation_enabled=true`，会拒绝上述诊断配置；本轮不修改它，也不把拒绝解释为诊断配置无效。候选 node 启动仍通过同一 `read_ros_config()`、`Config::validate()` 及 Runtime 检查参数。

在已加载 `/opt/ros/humble/setup.bash`、现用 SDK `install_r632/local_setup.bash` 且确认 Domain 0 的终端，人工单独启动候选；共享 launcher 锁被占用时直接停止，不启动第二个控制器：

```bash
flock -n -F "$HOME/.cache/fc01_operator/launcher.lock" \
  "$PREFIX/lib/jikl_fc01_system/fc01_system_node" \
  --ros-args --disable-external-lib-logs --params-file "$DIAG/runtime.yaml" \
  -p "socket_path:=$DIAG/control.sock" 2>&1 | tee "$DIAG/controller.log"
```

先从实际 node state 核对上述门全部为 false、未解锁，记录 controller/Agent PID。只运行诊断候选应保持 IDLE，不发送 ATTITUDE_READY、MODE、解锁、起飞或导航命令；不会产生预发送三次 context，不能用此诊断 profile 验收实机 Offboard 采用。

在第二个同环境终端填入相同的 `NEW`、`DIAG`，只读 STATUS 每 10 s 更新一次，再运行被动探针 180 s；探针沿用当前 DDS Domain，不打开串口、不发布控制话题：

```bash
(
  for n in $(seq 1 18); do
    /usr/bin/python3 "$NEW/native/tools/fcctl.py" --socket "$DIAG/control.sock" \
      --command status --json > "$DIAG/status-next.json" || exit
    mv "$DIAG/status-next.json" "$DIAG/status.json"
    sleep 10
  done
) &
FC01_DIAG_READER=$!
/usr/bin/python3 "$NEW/feedback_age_probe.py" --duration 180 \
  --status "$DIAG/status.json" --output "$DIAG/feedback-age.jsonl"
wait "$FC01_DIAG_READER"
```

观察结束后在候选终端 Ctrl+C 正常退出，保留整个 DIAG 目录，不重启 Agent。若出现其他控制源、门值不符、已解锁或非零推力，立即结束软件诊断并交由现场操作者处理，不进入后续动作。无需重复 UART 回环、CRC 接收或基础 DDS 通信测试。

按同一 controller PID 检查：位置/姿态回调源年龄及 RMW 差是否仍累积、被动探针是否同步累积、回调次数与收集预算是否有界、最大周期及 stall、锁存与拒绝原因。所有动作门关闭时，本次预期 target/heartbeat/VehicleCommand 无新增、推力为零；三次发布边界的行为由隔离的合成回归覆盖。不要仅以服务 active 或末次状态判定通过。真实持续过期或安全事件仍需拒绝，不得为了验收要求继续发布。

该观察只能验收 ARM64 软件运行和反馈消费。未确认 PX4 采用、动力响应、真实离地或自主飞行。若还出现积压，提交这一次完整报告及精确 PID 即可，先分析剩余证据，不重复整套硬件测试。

## 8. 合并与回退

开发分支 `dev/feedback-drain-20261107`，基于指定分支 `dev/offline-control-stability-20261107` 的 `97eb691`。修改已按作用拆分；合并前在目标分支审查依赖及本地编辑：

| 提交 | 范围 |
| --- | --- |
| `e8afaac` | AsyncLog 退出谓词与通知同步，独立于反馈消费 |
| `c285838` | 有界接收、关键观察保留、生产接收诊断和针对性安全回归 |
| `731b450` | 被动探针、隔离对照工具、明确诊断 UI、独立源码包目录支持 |
| 文档提交 | 本报告；完整交付 HEAD 记录在安装包旁的 validation JSON |

按表中顺序审查/cherry-pick，或在已包含基线的集成分支合并整个开发分支。此前离线时钟、完整周期预算和最终发布保护保持；没有 PID、推力、QoS、DDS V3 或 UART 配置差异。用户原有 `fc01_config_check.cpp` 空白编辑未纳入提交或包。

完整修改文件（23 个）：

```text
native/src/jikl_fc01_system/include/jikl_fc01_system/async_log.hpp
native/tests/test_async_log.cpp
native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp
native/src/jikl_fc01_system/include/jikl_fc01_system/receive_events.hpp
native/src/jikl_fc01_system/include/jikl_fc01_system/receive_schedule.hpp
native/src/jikl_fc01_system/src/fc01_system_node.cpp
native/tests/test_receive_events.cpp
native/tests/test_receive_schedule.cpp
tests/ros_offline_timing_smoke.py
build_navigation_bundle.py
feedback_age_probe.py
navigation.py
navigation_service.py
navigation_status.py
tests/test_feedback_age_probe.py
tests/test_navigation_diagnostics.py
tests/test_navigation_install.py
native/tests/ros_receive_ab/CMakeLists.txt
native/tests/ros_receive_ab/README.md
native/tests/ros_receive_ab/probe_smoke.py
native/tests/ros_receive_ab/receive_ab.cpp
native/tests/ros_receive_ab/run.py
docs/FC01_FEEDBACK_DRAIN_20261107.md
```

交付文件在本机 Desktop：`FC01_OrangePi_Navigation_20261107_feedback.sh`，附 `.sha256` 和 `FC01_20261107_feedback_validation.json`。这是可在 ARM64 本地离线构建的源码包，需要香橙派已有 Humble、匹配的 px4_msgs SDK 和编译工具；不是预编译 ARM64 二进制。默认解包目录独立为 `FC01_OrangePi_Navigation_20261107_feedback`，完整源提交、ZIP payload 和安装器校验值在外部 validation JSON 记录，避免包内自引用校验。校验方法：

```bash
sha256sum -c FC01_OrangePi_Navigation_20261107_feedback.sh.sha256
```

本轮没有自动部署。仅诊断时退出候选即可，原安装目录与服务配置保留。源码回退在干净集成分支按逆序 revert 上述代码提交，保留前序基线；如已由现场切换，再按已有人工流程恢复旧目录。回退不清除严重故障记忆，不强制杀死借用 Agent，不改变 UART 或飞控参数。
