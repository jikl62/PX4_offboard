# FC01 Stage 3：可靠性修复与验收

开发基线为 `f06715a70c6c070ecce25551ecb5de535e51b7a3`，实际启动时分支正确、工作区干净。所有修改仍在 `readability/fc01-chinese-20261111`，保留此前历史和证据；本轮只做本地提交。没有 push、merge、rebase、部署、服务操作、生产控制节点、Agent、串口、解锁或真实电机活动。测试中的起飞是合成反馈和内存 sink，不能当成动力学仿真或飞行验收。

## 修了哪些真实问题

本轮没有新确认 P0/P1；以下按 P2 工程可靠性问题登记。故障严重性没有从主机复现扩大成已经发生的空中失控。

| 问题 | 基线复现 | 最终处理与边界 |
|---|---|---|
| 探针最终证据写盘失败跳过 ROS 清理 | ENOSPC、EACCES 等注入及真实 EISDIR；Node 未销毁、Context 仍活跃 | 分离原证据写入与有序终结；写入、destroy、shutdown 每步失败仍尝试后续释放。保留业务原错；没有业务错误则首个终结错误向外抛，后续错误保留在异常链。 |
| 探针清理错误覆盖主错、误关已有 Context | destroy 抛错跳过 shutdown；回调异常被 finally 写盘错误覆盖；已有活动 Context 重复 init 后被关闭 | 仅取得本次 init 前默认 Context 不活跃的所有权；原先活跃的 Context 保留。显式记录本轮主异常，不依赖调用者可能正在处理的异常。单次默认 Context 合同不变，不承诺多线程调用者隔离。 |
| 视觉适配器部分构造丢失 Node 所有权 | 真实 Humble 基线 created=1、destroy=0；Context 关闭但保留 Node 引用仍能 get_name | 构造只完成 ROS 基类 Node；main 先持有对象，再执行原顺序 configure_adapter。异常时依次 remove、destroy、自有 Context shutdown、恢复两个信号处理器。当前 created=1、destroy=1、句柄失效。 |
| 视觉清理遮蔽主异常、误关调用者 Context | 控制异常/多项清理错误的有序注入及真实重复 init | 原 RuntimeError 等继续传播，原 cause/implicit context 和次生清理失败均可复核。SIGINT/SIGTERM 仍只置标志，不在回调中 shutdown 或抛异步异常。空闲等待 100 ms 不构成最坏退出时限。 |
| 视觉身份没有应用层资源预算 | 基线两个 1 MiB source_id 可进入缓存并处于 active；尚无实飞故障证明 | 身份最多 256 UTF-8 字节，拒绝 NUL 和非法 UTF-8；在首次应用复制、半包缓存、核心资格和 JSON 诊断前检查。拒绝撤销资格，保留旧有界身份及序号水位，不截断后冒充有效来源。消息 schema 不改。 |
| 原生日志初始化和分配异常 | F_GETFL/F_SETFL 失败留下 SIGPIPE=IGN；线程 EAGAIN 留下 O_NONBLOCK；append/queue bad_alloc 导致流 badbit 且无 drop 计数；析构分配异常退出 -6 | 构造失败回滚已改进程设置再传播原错；append/queue 内存不足作为有界整段丢弃并计数，push 成功后才提交队列字节数。注入 ENOSPC/EIO 后下一条日志仍可恢复。OS 若连回滚调用都拒绝，只能尽力恢复；全局堆耗尽仍不具备任意控制代码继续运行保证。 |

旧 `probe_cleanup.patch` 已独立复核，但直接应用不足以满足任务：嵌套 finally 仍会覆盖初始错误，也没有区分调用者已有 Context。此次正式实现因此比原候选更明确。

精确文件清单、修改原因及 SHA 见 [changed_files.json](changed_files.json)。原始失败、当前成功和独立审查记录的路径、摘要见 [verification.json](verification.json) 及 [evidence_index.json](evidence_index.json)。

## 字符串合同与兼容性

详见 [VISION_RESOURCE_CONTRACT_zh.md](VISION_RESOURCE_CONTRACT_zh.md)。256 字节沿用工程已有速度预览来源预算；这是明确新增的功能拒绝门，单独提交，没有混入等价重构。`jikl_fc01_system` 的五份 MSG、字段类型、Topic 和 schema_version 均不变。现有正常身份在 2400 条 C++ 精确文本记录、80 条 Python 完整规范 JSON 对照中逐字节相同。ASCII 256 字符、128 个 é 或 64 个四字节字符均处于边界，超出一个字节即拒绝。

应用门不限制 DDS 反序列化之前的内存分配，不能代替有界 ROS schema、DDS 配置或 OS 资源治理。若未来改成 bounded string，须作为独立消息版本迁移与视觉同学协商；本轮保持 V1 类型兼容。视觉默认仍 false，VelocityCommand 诊断仍不具有飞行权限。

## 控制状态和异常组合验证

`Core::step`、`Runtime::step`、`SystemNode::tick`、`RcNavigationStart::step`、publication_boundary、discrete_event_latch 的生产源码均未因本轮重构而改变；视觉输入校验及日志辅助实现的改变已单独登记。不为了让函数短而移动最终发布检查、浮点公式或状态更新。

同一驱动分别编译不可变 f06715a 与当前实现，严格比较阶段、事务、原因、hexfloat 输出和调用次序。数值容差为零。原完整动力链记录包含 707 份状态、588 对目标与心跳、一次上锁命令；三轮完整纯核心 1.5 m 生命周期也再次对照。节点诊断 PID 作为固定夹具输入，不修改真实进程所有权。

| 必须保持的不变量 | 本轮证据 |
|---|---|
| 危险事件不能被健康尾帧覆盖，Kill 不被普通更新清除 | 新增 25 种两危险事件夹健康帧；随后 12 帧普通反馈仍 Safety 且静默。实际节点旧接收/恢复回归同时运行，覆盖各来源好→坏→好、Kill 开关时钟、姿态/垂向 reset。 |
| 没有本轮授权不能产生动力；冷启动不恢复旧飞行 | 新增 256 种 armed/offboard/landed/land-fresh/rest/开关/Kill 组合，每组 80 步；无程序 Arm/Offboard、无非零推力、flight_started=false。这里是策略后模拟提交，实际双 sink 由另一驱动验证。 |
| RC 接管后不能自动夺回 | 新组合中的 Offboard 丢失锁存；原 RC 接管、before-ready、接收队列和重准备回归；只有新的安全地面证据及后续人工边沿可以开始新任务。恢复到 READY 不是自动起飞。 |
| 未知接地不授权动作、旧任务不污染新任务 | 原 ground/unknown/bounce/reset/time/recovery 拒绝矩阵；完整降落、上锁、复位与新窗口回放；320 轮 Command400 历史淘汰及因果水位回归。 |
| 过期控制目标不能因计划曾通过而继续发布 | 新三观察点 8³=512 组合，包括 stall、Offboard 丢失、Kill、failsafe、旧状态、竞争与 takeover；目标前拒绝无 sink，心跳前拒绝明确为部分提交。 |
| 非原子发送不能记成完整成功 | 新 10 组 Context/target/heartbeat 的 RuntimeError 或 bad_alloc；断言调用次数、stage、committed 和 sink_exception。目标调用抛错时不假定远端没收到。 |
| ACK 不能错配到新事务 | f067 同输入 request/ACK/IPC 严格差分；原迟到/歧义/重复 400、176 状态确认、历史淘汰回归重新执行，不能用 ACK 替代新状态证据。 |
| 突发保留 FIFO，溢出不能靠 ack 消失 | 新 1024 个十事件序列、容量八；逐项匹配前八次序，后两次拒绝，ack 后 overflow 仍粘性。原生产容量 32 的接收回归另测，未改容量。 |

这些是有界、确定性的组合和故障注入，不是所有输入/所有时序的数学证明。有限状态回放也不模拟空气动力学、真实 DDS 调度或 PX4 输出采用。

## 运行现象如何解释

| 现象 | 当前判断 | 实证和剩余边界 |
|---|---|---|
| CONTROL_LOOP_STALL | 现有安全保护 | 本轮最终边界 stale/stall 注入及整节点差分保持原拒绝；未解锁零流准备只有原有窄恢复。没有提高阈值或把故障改成继续输出。ARM64 p99/max、SD/温度/调度压力仍需实测。 |
| HEARTBEAT_FINAL_CONTEXT_CHANGED | 目标已调用后的最终拒绝 | 属于非原子过程，不能回滚目标或宣称整对成功。新组合逐点验证，保留部分证据并走原边界锁存。 |
| PARTIAL_OR_FAILED_SETPOINT_PUBLICATION | 现有发布异常保护 | 原节点锁边界、清当前输出、不发关联动作；原节点 sink 异常测试和新运行/分配异常注入均通过。真正 RMW/PX4 收到多少，必须查同程 ULog。 |
| 垂向反馈异常 | 独立监测与原硬有效性保护 | 速度差仍只诊断；非有限、时间回退、reset、样本间隔和位置积分残差继续拒绝。实际节点 40 Hz 输入/20 Hz 控制回归重跑，没有恢复成速度差独立停机。 |
| Agent 断开、Supervisor 退出、残留 socket | 原所有权及恢复合同 | 原 transport-loss/reuse/backoff/live-native/socket owner、父进程信号、真实本机 IPC 测试纳入全量。只用本地假进程/临时 socket；不代表真实 XRCE 链路重连和 PX4 failsafe 已验收。 |
| 磁盘和内存资源异常 | 本轮确认并修复特定路径 | probe 实际 EISDIR、受控 ENOSPC 等和 AsyncLog 真实 C++ 分配失败注入。没有填满用户磁盘或耗尽系统内存；可选同步 CSV 仍有延迟验证债。 |
| 高频/延迟/乱序/跟踪丢失 | 原时效和撤销保护加应用预算 | 配对超时、重复/乱序、时间回退、健康丢失和资源撤销被测试；真实 ROS 新增 ASCII/UTF-8 两轮各自拒绝并恢复。不是任意频率/恶意网络负载的资源上界。 |

## 测试、失败与证据

最终 Python 根目录 534 项、native 41 项，共 575 次测试方法执行通过，无跳过；Debug CTest 80/80、9 个独立 ASan/UBSan 程序通过；另重跑 11 项打包/损坏拒绝回归。完整命令以 verification.json 和证据 ZIP 为准。Python 原生入口与 CMake 均实际执行 C++ 程序，不能把两次执行加成独立用例数；Sanitizer 也按独立程序去重。TSan 重新编译成功，运行仍以 unexpected memory mapping、退出码 66 失败，未记为竞态检查通过。

本轮没有隐藏修复过程中的问题：独立复核先后抓到异常链丢 implicit context、资源拒绝丢序号水位、probe 误读调用者异常三处初稿回归，均新增明确测试后修正。第一次全量根目录 534 项只有原生聚合入口失败：日志注入测试少了 Python runner 的链接参数；CMake 的 80 个程序已经通过。随后补齐链接并计入二进制缓存键，重新跑完整根目录。最初过宽格式扫描还包含了历史反例；这些旧证据保持原字节，正式检查沿用 Stage2 活跃源码集合加全部 Stage3 改动。

Humble 构建在独立缓存目录完成；安装后的 35 份头、4 份 Python、5 份 MSG 与当前源码字节匹配。505 条消息各两次，共 1010 次字段/类型/浮点位模式 CDR 往返通过；按 schema 识别 padding，未要求无语义填充字节稳定。隔离 localhost 域先预检没有 /fmu，再运行只读适配器与诊断工具。

主机基准每项 3000 次：只记录同步序列化/诊断/转换/校验开销，不含 DDS 排队、实际控制循环或磁盘。x86_64 结果、源码身份、p50/p95/p99/max 和 RSS 在 ros/host_performance.json，不能套用于 Orange Pi。

## 剩余问题的分级与边界

本轮未留下已确认但未修复的 P0/P1/P2 逻辑缺陷。该结论只覆盖已审查和可复现路径，不是不存在未知缺陷的保证。

| 项目 | 分类 | 处理状态 |
|---|---|---|
| Core/tick/Runtime 等仍有较长协调函数 | P3 结构债 | 12 份逐文件记录保留；不为缩短代码改变阶段/早返回/发布顺序。 |
| 可选同步 CSV、日志慢端和板上高负载时延 | 性能验证缺口 | 没有确认导致实际飞行故障；须 ARM64 隔离测量。 |
| DDS 可先分配原始无界字符串 | 已知资源边界，未证明控制故障 | 应用预算已修；线上报文预算需独立协议/DDS资源方案，不能称本轮已封顶进程内存。 |
| TSan 运行环境不能初始化 | 工具验证缺口 | 明确失败；竞态检测未完成。 |
| 飞控真实采用、视觉质量、XRCE重连和热/存储负载 | 硬件验证缺口 | 主机合成通信与有限回放不能代替。 |

## 结构债和下一步

上一阶段 12 份部分完成文件仍逐项保留维护债。本轮实际改进 navigation_probe 的证据/清理责任、视觉 Node 生命周期及异常处理，并在 vision_navigation 入口增加资源门；其余 10 份部分完成文件不强制拆分；Core、Runtime、tick、RC 的阶段时序、同帧早返回、ACK 歧义和非原子 sink 仍显式可见。没有把修复数量冒称成“22 份全解决”。历史 source_coverage 和 Stage2 报告保持原样；[structural_debt.json](structural_debt.json) 逐项记录这 12 份文件的当前状态、SHA 与剩余职责，新文件清单只描述本轮。

没有发现阻止下一阶段隔离、无动力 ARM64 构建的已确认 P0/P1 代码问题。仍未具备 ARM64 编译、性能和实飞通过证据；主机没有 aarch64 编译器/QEMU，也未连接 Orange Pi。准备步骤和准入边界见 [ARM64_NO_POWER_zh.md](ARM64_NO_POWER_zh.md)。本结论只支持推进无动力验证，不是实飞放行。

需要视觉同学确认 256 UTF-8 字节身份合同、稳定 source/reset/sequence 规则，提供真实 Odometry 与 Health 同时戳样本、坐标方向、内外参/杆臂/世界对齐、时间偏差及抖动、米制尺度、质量/重定位/reset、可解释的协方差。仍缺实机相机/IMU 同步、标定样本、频率/延迟/跟踪丢失记录和 ARM64 压测；外部视觉并未直接进入 PX4 EKF。需要真实 PX4 日志才能核对目标采用、姿态振荡与恢复执行。

接下来优先：ARM64 单作业干净构建及无硬件性能测量；真实视觉录包合同/资源压力；同程导航报告与 PX4 ULog 对齐。继续在独立分支开发，是否推送由用户决定。

本轮独立提交（完整哈希及顺序见 verification.json）：`3fbb3f7` 探针清理，`9e9d739` 视觉生命周期，`769ad51` 探针异常来源修正，`2af4ede` 日志资源异常，`522923c` 状态/发布组合验证，`0e1937a` 视觉身份预算，`e8a856a` 原生测试入口链接。最后一份交付元数据提交以本分支 Git 历史为准，文档不自引用自身提交哈希。

证据归档：[evidence_curated.zip](evidence_curated.zip) 含关键原始异常、审查、命令和日志摘录；[evidence_index.json](evidence_index.json) 记录其 SHA 与完整包位置。完整 245 份文本（含约 80 MB 原始轨迹）压缩为约 2.45 MB，仅存开发机 cache，不随源码安装包携带。最终源码包只在 cache 生成并逐字节核验，不运行提取器或部署；该最后核验结果为 cache/final_integrity_verification.json，时间上晚于固定证据包，不冒称已包含在 ZIP 中。
