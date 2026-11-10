# 152318 现场报告：接管后恢复阻塞与接收延迟分析

分析输入：`FC01_navigation_report_20261021_152318_747738.zip`，解包目录 `/tmp/fc01_report_152318_b2udy6qr`。本文件是包外审查记录，不应加入部署 payload。以下结论限定于报告内保留的本次会话；日志时间为设备记录的 UTC。

**结论：本次先发生 Kill/Termination 接管，约 106 秒后才首次观察到 local/attitude 源过期。后续存在逐渐增长的 RMW→callback 延迟，证据支持接收队列积压，不能把 `DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE` 直接解释为时钟跳变。恢复还独立受 `at_rest=false` 阻塞，修复接收积压不等于已经满足重新准备条件。**

## 会话与证据范围

- `status.json`、`status_after_capture.json` 与 `REPORT_CAPTURE.json:270` 对应 `boot_id=f8742412-512b-498e-abfb-ffbd08493fcf`、supervisor PID 725、controller PID 2431、agent PID 2429、`entry_version=20261110.4`。报告捕获前后状态内容未变；捕获耗时约 0.239 秒。
- `journal.txt:2` 记录该 supervisor 以 `.4` 启动；`service.txt:3`、`:10`、`:11` 对应本次运行服务、agent 2429 与 controller 2431。运行入口版本为 `.4`，不能把本报告当作后续修复版本的实机验证。
- 三份 controller 日志解析出 557 条 `state`、562 条 `phase_event`、1 条 `offboard_stream_edge`；全部保留 state 的 PID 都是 2431。state 的 monotonic 范围为 108.010297–386.995231 秒，共 278.984934 秒。启动记录已经轮转掉，不能从保留日志重新验证本进程的完整启动输出。
- `probe.json` 属于另一 boot `f6112f61-58c2-4b75-9912-407130e035ab`、agent 2407；`invoked_entry/ACTIVATION_STATE.json` 指向 controller 4835、agent 2407。它们不是 controller 2431 的当前准备或源时序证据。目录中的构建/部署材料也不能替代当前进程身份。
- 设备日志中的日期是 2026-10-21，发布版本名包含 20261110。版本命名、其他 boot 的材料和当前会话的时间演进必须分别判断；本次保留数据未观察到当前会话的 host/source 时间回退。

## 先后顺序

| 事件 | monotonic 秒 | 证据位置 | 观察到的事实 |
| --- | ---: | --- | --- |
| DESCENT→SAFETY | 170.298412 | `controller.log.2:256` | 首故障和最新故障均为 `OPERATOR_TAKEOVER_LATCHED`；ownership 为 `EMERGENCY_KILL_NO_ROUTINE_DISARM`；仍 armed、Offboard，local/attitude 均新鲜。 |
| 首个保留 SAFETY state | 170.750509 | `controller.log.2:258` | `takeover_reason=KILL_OR_TERMINATION`；local 源年龄 0.038375 秒、RMW→callback 0.009206 秒，尚无源过期。 |
| 已未解锁、退出 Offboard 的区间开始 | 264.614290 | `controller.log.1:139` | nav_state=15、armed=false、landed=true、Kill 开关 OFF；`at_rest=false`，当前恢复原因是要求接地静止。 |
| 首次记录源过期 | 276.161854 | `controller.log.1:185` | local 源年龄 0.506735 秒、attitude 0.501192 秒，均超过 0.5 秒阈值；两者 RMW→callback 分别为 0.459100、0.481607 秒。 |
| 报告当前状态 | 386.493885 | `status.json:1` | local 源过期，故障仍为可恢复 OPERATOR；没有后续未知故障把恢复永久锁住。 |

`journal.txt:37`–`:41` 另记录本次 ARMED_GROUND、THRUST_RAMP、ASCEND、上升超时下降、OPERATOR 故障的顺序。15:17–15:18 的同 PID 早先任务在 `journal.txt:14`–`:25` 有各自接管与重新准备记录，不能与这次任务合并成一次失败过程。

因此，后续接收过期不能倒置为本次 Kill 接管的已证实触发原因。保留证据只支持上述先后关系；物理操作过程与机体运动仍需以本次实际观察为准。

## 当前具体过期来源

来自 `status.json:1`，时间为设备 UTC 15:23:18.059735 / monotonic 386.493885 秒：

| 源 | 超时阈值/秒 | 源 envelope 年龄/秒 | receipt 年龄/秒 | RMW→callback/秒 | 源判断 |
| --- | ---: | ---: | ---: | ---: | --- |
| status | 2.5 | 0.131520 | 0.100655 | 0.004417 | VALID |
| land | 2.5 | 0.950277 | 0.902526 | 0.018009 | VALID |
| local | 0.5 | **0.535409** | **0.000225** | **0.497330** | **ENVELOPE_EXPIRED** |
| attitude | 0.5 | 0.477119 | 0.000276 | 0.422885 | VALID，接近阈值 |
| flags | 2.5 | 0.539245 | 0.501467 | 0.023915 | VALID |
| battery | 1.0 | 0.491018 | 0.401312 | 0.044164 | VALID |

local 原始 sample 年龄为 0.536524 秒；envelope 与 sample 仅相差 0.001115 秒。刚发生的 callback 并不代表源数据刚生成：这个 callback 拿到的消息在调用前已经等待约 0.497 秒。attitude 同样呈现“callback 刚执行、消息已有数百毫秒”的情况。

557 条 state 的 source_timing 汇总：local 有 138 条 `ENVELOPE_EXPIRED`、6 条 `SAMPLE_EXPIRED`；attitude 有 63 条 `ENVELOPE_EXPIRED`、2 条 `SAMPLE_EXPIRED`。status、land、flags、battery 在这些 state 中全部为 VALID；所有源的 receipt 判断均为 VALID。

## 积压增长与接收轮次

SAFETY 区间共有 432 条 state，覆盖 216.244722 秒。其中 428 条 `receive_stop_reason=SAFETY_EVENT` 且 `receive_passes=1`，占 99.1%；其余 4 条为 IDLE，并执行 2 或 3 轮接收。非 SAFETY 的 125 条 state 中有 93 条 IDLE，接收能够执行更多轮次。

SAFETY state 的 local `callbacks_this_cycle` 有 430 条为 1、2 条为 0；attitude 有 431 条为 1、1 条为 0。该区间有 4310 个控制 tick，local 接受 4309 条消息、attitude 接受 4310 条，约等于每个控制 tick 各取一条。接收速率分别约 19.9265 Hz、19.9311 Hz；按接受消息的源 timestamp 跨度计算，消息序列约 19.97 Hz。这种很小的速率差会逐渐积累等待时间。

以下为各 20 秒区间的中位数；区间采用设备 monotonic 秒：

| 区间 | local 源年龄/秒 | local RMW→callback/秒 | attitude 源年龄/秒 |
| --- | ---: | ---: | ---: |
| 170–190 | 0.122613 | 0.086598 | 0.119561 |
| 190–210 | 0.209070 | 0.169246 | 0.205112 |
| 210–230 | 0.284091 | 0.247530 | 0.281301 |
| 230–250 | 0.354068 | 0.318016 | 0.354060 |
| 250–270 | 0.436916 | 0.399620 | 0.432936 |
| 270–290 | 0.503020 | 0.466229 | 0.483227 |
| 370–390 | 0.509420 | 0.472025 | 0.493660 |

日志显示延迟先持续增长，再在约半秒附近波动。源码配置 `SensorDataQoS().keep_last(10)`；20 Hz 消息的 10 深度队列与这个时间尺度相符。这里有明确的本地 callback 等待证据，仍不能凭日志把 XRCE/串口/RMW 的全部延迟贡献精确拆开，也不能声称已经观察到真实 DDS 队列内容或逐消息丢弃过程。

所有 557 条 state 中，`host_clock_event_count=0`、`cycle_stall_count=0`、`publication_fault_latched=false`；各源 stamp regression 均未出现。状态与 land、flags 等源仍按约 1–2 Hz 更新，local/attitude 也持续接收。这不符合“所有源共同停止更新”或“本次已观察到时钟跳变”的解释。

## 两个接收调度触发条件的只读复现

现有类型/API fixture 使用真实 `SystemNode`、Runtime/Core、接收判定函数与 `ReceivePassSchedule`，在 `/tmp` 中完成两种独立验证，未修改源码：

1. Core 已处于 SAFETY、相邻 local 样本稳定且全部有限；落地后的 z 相对旧 flight 的最后控制样本偏移 0.6 米。相邻事件判定为 false，但旧飞行基线仍使 `local_requires_control_step=true`；接收调度在一轮后停止为 SAFETY_EVENT。
2. 垂直数据稳定、XY 不可用且 x 为 NaN。`local_receive_event` 对每个样本都返回 true，而控制提示可以为 false；接收调度仍在一轮后停止为 SAFETY_EVENT。

严格 C++17 警告检查与运行通过，输出分别为 `adjacent_event=0 / stale_flight_control_hint=1` 和 `adjacent_event=1 / stale_flight_control_hint=0`，两者均 `passes=1`。这验证了源码中的触发机制；它没有运行真实 ROS/DDS，不能替代实机验证，也不能单独证明报告里的每个事件都由哪一个条件产生。

接收追赶修复应继续保留已排队的不安全 payload、接收预算和轮次上限、源 TTL，以及独立的新鲜地面证明。不能通过延长 TTL、忽略异常事件或直接清除接管历史来掩盖积压。

## 地面静止与原始有效位的限制

不能把整个 SAFETY 区间写成 `at_rest` 从未为 true：早期有 16 条 true（`controller.log.2:272`–`:302`，monotonic 174.264877–181.794527 秒），当时仍有 Kill、解锁/Offboard 等独立阻塞。

但在已未解锁且退出 Offboard 的全部 245 条 state 中（monotonic 264.614290–386.995231 秒），`at_rest` **全段为 false**。从首次源过期到日志结束的 222 条 state 中，`at_rest` 也 **全段为 false**。当前 `status.json` 同时是 landed=true、at_rest=false、stationary_seconds=0。即使本地接收延迟消除，仍需真正的新 land/rest 反馈及完整静止驻留，才能完成恢复。

状态 JSON 的 `z_valid`、`vz_valid`、位置与速度输出会按 `local_fresh` 屏蔽。报告中 z/vz 变为 false 的 144 条 state，正好与 local_fresh=false 的 144 条对应。当前过期状态中看到 `z_valid=false` 或位置 null，**不足以断言 PX4 原始 payload 的 z_valid/v_z_valid 已经失效或 z/vz 是 NaN**。同样，XY/heading 的可用标志无法单独证明原始数值恰为 NaN；本报告没有独立的 raw payload_valid 字段。

## 可确认与尚未确认

- 已确认：本次运行报告为 `.4` 的 controller 2431；首故障与最新故障始终为可恢复 OPERATOR；Kill/Termination 接管早于源过期；local/attitude 的本地 callback 等待逐渐增长；当前恢复同时缺少新鲜 local 和静止证据。
- 未确认：现场每个接收事件的具体触发条件、全部传输延迟的分解、PX4 原始 z/vz 有效位、为何 land detector 后段始终 at_rest=false，以及后续修复版本在该实机的实际效果。上述信息不能从不同 boot 的 probe/activation 材料补齐。
- 本分析没有修改控制增益、扩大恢复授权或执行设备命令。后续应使用当前会话的新 report/status 验证接收追赶后的源年龄、轮次和地面证明进度；已有报告的旧会话材料不能作为当前运行状态。

复核用临时产物：`/tmp/fc01_review_152318_parsed.json`（保留原文件/起始行号），解析器 `/tmp/fc01_review_uploaded_recovery.py`，只读机制复现 `/tmp/fc01_source_queue_fixture_audit.py` 与 `/tmp/fc01_source_queue_audit_yskq9jsi`。原 controller 日志存在分块/时间前缀拼接，统计先去除各片段前缀再按 JSON 对象解码；不能把单个物理行直接当成完整状态 JSON。
