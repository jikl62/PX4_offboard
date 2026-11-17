# FC01 第二阶段：职责拆分与可靠性审查

本阶段从 `5198d33f25aaaf2abc7e4b078169ae84d5c4b2c4` 开始，继续使用 `readability/fc01-chinese-20261111`。本地开发按模块提交，未推送远程、未合并 main、未部署，也未运行生产控制节点、Agent、串口或真实动力试验。这里的起飞和电机输出均为主机夹具中的合成反馈和内存 sink。

本轮实际改变了函数结构，不能继续用上一轮 token/AST 全文件相同作为结论。只有视觉工具的退出生命周期存在独立、明确的行为修复；控制器、权限、参数和发布政策的抽取以严格动态对照、原回归及独立阅读验证。

## 正式 22 份文件的处理结果

清单直接来自上一阶段 `source_coverage_20261111.json` 的 `structural_refactor_pending`。9 份完成清单中目标职责的拆分，12 份部分完成且保留具体结构债，1 份经审查保持原字节。21 份进行了真实函数抽取，没有仅增加注释却登记为结构完成的文件。

“完成”只表示本次列出的混合职责已分离，不表示文件永远不再需要维护。上一阶段覆盖、人工 SHA 记录和验证报告属于历史快照；当前文件以本目录的新 SHA、指标和验证记录为准。

| 文件 | 本轮处理 | 仍保留的原因或边界 |
|---|---|---|
| `launcher.py` | 部分；控制台输入、显示、入口配置与资源准备分开 | 控制台循环及部分失败时的资源所有权仍由原协调器持有 |
| `native/tools/fcctl.py` | 部分；一次性命令门、台架预检、显示签名与帮助分开 | CLI 操作、ACK 等待与各协议水位仍需顺序协调，`run` 仍较长 |
| `native/tools/vision_odometry_adapter.py` | 目标完成；时钟、标定、话题、QoS、接线、消息映射、原因日志及退出分开 | `_adapter_node_type` 的 240 行包裹类定义，不把外壳长度冒称为新控制算法；构造失败清理缺口另列 |
| `navigation.py` | 目标完成；状态读取后的当前/历史显示与恢复观察分开 | 保留每个故障文本、字段及输出字节 |
| `navigation_probe.py` | 部分；地面与 RC 订阅创建分开 | 证据等待、已观察成功水位与最终清理仍协调；写盘异常缺口保留为候选 |
| `navigation_report.py` | 目标完成；预算/去重/安全文件捕获、入口来源和命令捕获分开 | 报告只读行为、文件预算、NOFOLLOW 和采集顺序保持 |
| `navigation_restore.py` | 部分；备份核验、验收等待、失败恢复分开 | `changed/started` 必须在实际动作前更新，异常补偿不能丢水位 |
| `navigation_service.py` | 目标完成；解析、配置检查、运行装配与分派分开 | 原确认要求、模式及配置生成不变 |
| `standby.py` | 部分；状态发布所有者、单次退出、退避、Agent/控制器创建分开 | 传输先于控制器，清理等待先于结束自有 Agent，启动失败的部分所有权仍由协调器掌握 |
| `attitude_altitude_core.hpp` | 部分；程序 Arm、斜坡、恢复来源记录/原因分开 | `step` 仍 795 行；公共安全门、同帧下降、着地/上锁/释放不能盲拆 |
| `attitude_types.hpp` | 目标完成；准备/恢复合同、动力包络校验分开 | 跨字段关系和首异常顺序保持，未改为表驱动 |
| `authority_telemetry.hpp` | 目标完成；时钟/来源门与轴、开关载荷提交分开 | 无效时钟 Kill ON 可否决、OFF 不能清锁的非对称性保留 |
| `publication_boundary.hpp` | 原字节保留，未抽取 | 初查、目标前、心跳前是三个不同观察点；目标已经发出后的异常不能当作未发布 |
| `publication_guard.hpp` | 部分；目标姿态匹配、RC 准备零流合同分开 | 总策略 125 行，阶段例外和首拒绝次序继续显式保留 |
| `rc_navigation.hpp` | 部分；预发送、等待导航配对和阻断动作分开 | 地面参考 87 行仍保留独立源推进；过期先拒绝，再配对，不在 armed 时重建参考 |
| `runtime.hpp` | 部分；姿态请求、ACK 结果、地面步、终态取证、新动作跟踪分开 | 过滤→Core→状态终态→新请求顺序保留；400 复用歧义和有界历史不能清零 |
| `unix_server.hpp` | 目标完成；accept、一次发送、一次接收分开 | 原非阻塞字节界限、序号、先 drop 再回调及 inode 所有权保留 |
| `velocity_core.hpp` | 部分；坐标转换、滤波及制动输出分开 | 历史通用/仿真路线的迟到 Arm、任务托管和阶段分派保留 |
| `vertical_controller.hpp` | 目标完成；输入拒绝、积分接受职责分开 | 高度→速度→试积分→raw→可达→接受/退积分→重算 raw→slew 的浮点次序保留 |
| `vertical_feedback_guard.hpp` | 目标完成；速度差诊断、位置积分窗口分开 | 速度差仍只诊断；位置残差、硬数据故障和重复戳规则不变 |
| `vision_navigation.hpp` | 部分；已接受输入连续性校验分开 | 目标捕获、外环及中间非有限拒绝仍有状态顺序，未改成隐含原子计划 |
| `fc01_system_node.cpp` | 部分；静止窗口、两路线反馈映射、事务 JSON 分组 | 构造/回调注册未拆；`tick` 保留危险事件、三次边界、两个 sink、动作及恢复的可见顺序 |

逐文件 SHA、目标函数、嵌套、职责和保留理由见 [refactor_coverage.json](refactor_coverage.json)。没有为凑满 22 份而重写控制器。

## 可读性具体改变

物理行包含函数范围内的注释和布局。C++ 嵌套使用 Clang 14 AST，连续 `else if` 也计入嵌套，所以不能把这个数直接理解为人工缩进深度。新辅助函数说明参数、返回值、状态所有者、适用的时间/坐标单位、安全边界和调用顺序；协议名称、错误字符串及公开消息身份保持原样。

| 函数 | 原行数 → 当前 | 职责变化 |
|---|---:|---|
| `render_status` | 192 → 62 | 当前状态、历史显示、恢复诊断不再挤在一个表达流程 |
| `standby.supervise` | 225 → 120 | 状态发布对象持有去重/历史，退出清理和重试分别命名 |
| `navigation_service.main` | 182 → 50 | CLI/检查/运行与分派分开；嵌套仍为 6，未声称所有复杂度都下降 |
| `Runtime::request/ack/step` | 205/150/259 → 137/112/151 | 分派、结果分类、Core 后取证和新事务分别命名 |
| `RcNavigationStart::step` | 201 → 109 | 原预发送、导航配对状态保持独立次序 |
| `Core::step` | 933 → 795 | Arm 与斜坡分支抽出，公共健康门和其余阶段仍集中 |
| `VerticalController::step` | 83 → 71 | 公式仍是同一数学管线，抽出拒绝和积分决策 |
| `VerticalFeedbackGuard::observe` | 81 → 40 | 硬输入与水位、速度差、位置积分分开 |
| `SystemNode::snapshot/state` | 171/191 → 71/148 | 采样和图仲裁留在入口，反馈映射及事务字段有明确名字 |
| `SystemNode::tick` | 379 → 360 | 仅抽地面静止窗口；实际 sink 及异常范围未移动 |

这些改变主要减少一个入口同时管理的责任，不承诺行数越少越安全。完整指标见 `python_metrics.json`、`cpp_metrics.json`、`runtime_node_metrics.json`、`vision_metrics.json`。独立审查还使用 Clang 原始词法器对 16 个移动代码块展开后逐项核对，未发现语句顺序差异。

## 行为如何对照

同一份确定性输入分别编译/运行不可变基线与当前生产代码，不复制控制算法作为替身。ROS API、时钟和实际飞控反馈在节点测试中使用受控夹具；真正 `SystemNode`、接收观察、Runtime、Core、发布政策及序列化均执行原生产实现。

| 类型 | 本轮证据 | 判定 |
|---|---|---|
| 飞行阶段、推力、姿态、RC、故障和恢复 | 14,626 行核心记录，含三轮完整 1.5 m 保持/下降/上锁/释放、12 轮 RC 场景、6 组程序 Arm 场景 | 全部字节一致，double 使用 hexfloat，容差 0 |
| 请求/ACK/事务/本机 IPC | 冷启动请求矩阵、错误参数/优先级、重复/迟到/偏斜 ACK、真实临时 Unix 分帧/背压/业务异常 | 全部字节一致，事务终态和原因不得宽松比较 |
| 发布政策与非原子 sink | 全阶段×3 路线×26 类证据×6 类推力，共 10,764 条政策；另 15 组重采样/异常 | 原始 double 微小非零、三次 context、目标→心跳及部分提交记录严格一致 |
| 地面节点接收/恢复 | 1,350 份完整状态及完整零流载荷，含目标/心跳异常和周期超时 | 字节一致；仅临时源码的诊断 PID 输入固定为 4242，真实进程锁仍用真实 PID |
| 非零真实节点流程 | 707 份完整状态、588 对目标/心跳和一次上锁命令 | 所有目标/模式/命令字段精确记录，逐轮核对目标→心跳→可选动作 |
| Python 服务 | 9 组、119 个场景：状态字节、ZIP、配置、激活补偿、监督、控制台、ACK、只读探测 | 规范 JSON 字节严格一致，不用 Python 的 `True == 1` 弱相等 |
| 视觉接口抽取 | 5198d33、独立退出修复、当前版本，六场景共 18 次执行 | 12 组参数/QoS/消息/回调/日志记录严格一致 |
| 有意行为差异 | 独立视觉退出修复 | SIGINT/SIGTERM 只置停止标志；回调返回后移出 executor，再销毁节点、关闭 context、恢复处理器 |
| 未验证 | ARM64 性能、真实 PX4 接收/采用、实际动力稳定性、真实视觉标定/尺度 | 不由主机通过结果推断 |

数值允许误差这一类别本轮没有用到。消息 CDR 对齐填充可能不同，不属于字段差异：安装验证计算完整 schema 的字段位置，精确检查每字段/类型/浮点位模式，并对填充和有效字段做正反控制，未放宽权限或控制输出比较。

真实节点完成上锁后，同时切回 Stabilized 并复位开关，会先经过原离散事件观察并锁存 `OPERATOR_TAKEOVER_LATCHED`，随后满足新的地面窗口后恢复 `RC_READY_WAIT_ARM`。新测试验证了这条接收路径，没有套用纯 Core 夹具“直接 Idle”的假设，也没有为缩短恢复流程绕过锁存。

## 已确认的问题与处理

详见 [KNOWN_ISSUES_zh.md](KNOWN_ISSUES_zh.md)。本轮没有确认新的 P0。视觉停机竞态按独立修复提交；其余改变行为的处理保持未应用候选或待合同设计，未混入等价重构。

旧候选只改变 SIGINT，重新测试暴露 SIGTERM 关闭上下文的竞态。直接在 Python handler 抛 `KeyboardInterrupt` 的第二候选也在 6 轮中失败一次。最终 handler 只置标志，空闲 `spin_once` 等待上限为 100 ms；已经运行的回调必须先返回，**100 ms 不是最坏退出时延承诺**。普通 `RuntimeError/RCLError` 不被吞掉。重复停止信号不打断清理，处理器在异常路径也恢复。

## 实测与复现

最终状态、数量、命令、日志摘要和失败记录以 [verification.json](verification.json) 为准，原始证据封装在 `evidence.zip` 与 `evidence_traces_*.zip`，逻辑路径、SHA 和重复内容别名见 `evidence_index.json`。证据只含文本、JSON、脚本和补丁，不含构建缓存、ELF 或部署包。相同字节仅存一次，索引保留各原始路径。8 份中间输出、临时指标源码或未比较的启动诊断 stdout 仅留在执行缓存，索引明确登记 SHA、大小与原因；归档包含全部参与差分的状态及 sink 记录，不含所有原始 stdout 诊断。每个 ZIP 小于既有 1 MiB 限制，没有放宽安装器上限。以下命令均在开发主机执行：

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m unittest discover -s native/tests -p 'test_*.py' -v
cmake -S native/tests -B /tmp/fc01_stage2_tests -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01_stage2_tests -j2
ctest --test-dir /tmp/fc01_stage2_tests --output-on-failure -j2
```

最终全量 Python 回归：根目录 486 项、native 41 项，共 527 项通过，无跳过；Debug CTest 76/76、10 个独立 ASan/UBSan 程序、125 份 C++ 和 107 份 Python 格式检查通过。Humble 干净构建、安装后 5 项视觉和 11 项目标速度只读通信检查通过。TSan 编译成功但运行环境初始化失败，不计为通过。

前两次全量 Python 验收先后暴露过期源清单、夹具阶段假设和证据载荷超限；本轮更新真实摘要、纠正基线同样失败的夹具假设、去重并精简交付证据后重新跑完整套件通过，没有放宽打包器的 1 MiB/8 MiB 边界，也没有因此改动控制源码。失败日志与最终通过日志均在索引中。

历史差分在 Git 基线可用时运行；本轮实际执行没有跳过。源码安装包没有 Git 历史时不能声称历史差分已完成。Humble 干净构建、隔离域预检与通信脚本、Sanitizer 参数及安装校验脚本的完整命令见证据，不运行生产控制节点。

11 份源 YAML 的字节与基线完全相同，生成导航 profile 仍为 `0.50 / 0.04 / 0.80`、`1.5 m`、`arm_enabled=false`、`visual_navigation_enabled=false`。`FC01_RELEASE.json` 的源/载荷摘要随当前源码更新，仅描述开发源码，不生成或部署新的 dist 安装器，不冒称 ARM64 验收。

## 后续最值得做的三件事

1. 给主状态机剩余的同帧阶段转换、故障注入和不变量建立更系统的状态空间验证，再考虑拆 `Core::step`、节点 sink 编排和 Runtime 的剩余段落。当前有限轨迹相等不证明所有输入组合相等。
2. 将视觉接口合同与资源预算落实：最大字符串/消息大小、时钟同步、坐标外参/杆臂、协方差变换、尺度、reset 和来源身份，需视觉同学提供实际样本及标定。`Odometry` 是自身估计，`VelocityCommand` 仍只是目标速度诊断；当前默认没有视觉飞行权限。
3. 在 ARM64 上做无动力构建与负载/退出/日志压力验证，再用同程 PX4 ULog、导航报告、配置身份核对实际输出采用及恢复。没有这些证据不能从 x86_64 假消息回放推断真实飞行稳定。

可选 CSV 同步 flush、日志耗时、DDS 图与消息积压在真实负载下的上界仍需测量；本轮没有编造这些项的硬实时保证。保持所有后续开发在独立分支，是否推送由用户决定。

## 本地模块提交

独立最终交付复核未发现阻断项；22 份覆盖身份、31 份已改源码身份、全部配置和全部差分字段均已核对。最终报告更新后另做打包专项回归和精确载荷核对，避免报告自身让源码清单过期。

末尾两次 GitHub 只读查询分别遇到 TLS 断连和 443 连接超时，未获取远端结束哈希；不据此宣称远端哈希已再次核验。本轮没有任何 push 或远端修改，后续是否推送仍由用户决定。

以下五项生产与测试提交都位于当前独立开发分支，均以不可变基线为祖先；最后另提交本目录报告及源码摘要。完整提交身份同时见 `verification.json`。

- `df735d6a57def5873c7bdd331e118962baa428ef`：fix(vision): finish callbacks before signal shutdown and restore handlers
- `fd595bfee4f5e10c3f7732e082ebf7de761a2176`：refactor(python): separate service lifecycles and diagnostic responsibilities
- `a73e98e815b6b16c9f1f384a4dd1cbb4672726bc`：refactor(vision): separate ROS configuration callbacks and executor lifecycle
- `e4fc5e753dc1a7d10a849fc84b4c0b6f2d17639f`：refactor(control): extract ordered control phases without changing numerical behavior
- `fc13565f4d59ed30f94d73e1db70a4eb3e251cfe`：refactor(runtime): separate transaction and snapshot responsibilities with exact traces
