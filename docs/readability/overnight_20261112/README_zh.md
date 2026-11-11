# 今晚开发基准与逐文件可读性审查：阶段 1/7

本阶段以实际 Git 状态为准：分支 `readability/fc01-chinese-20261111`，HEAD 为 `5b86eeb4094ada009903c0140f66289cfe89411e`，开始时工作区干净。Stage 3 的八个本地提交和全部既有证据保留。本阶段只新增本目录报告，不修改源码、配置、发布清单或历史报告，不提交、不推送、不部署，不启动控制节点或真实飞控程序。

## 怎么用这份交接

- [BASELINE.json](BASELINE.json)：初始 Git 身份、关键参数、各 YAML 的显式值、最近完整验证及包体基准。
- [file_index.tsv](file_index.tsv)：323 份文件的路径、评估、定位行和后续任务编号；在 VS Code 搜文件名，再跳到对应行。
- [TASKS_2_TO_7_zh.md](TASKS_2_TO_7_zh.md)：六个后续阶段的输入、具体动作、行为保护与验收门。它是建议处理顺序，若后续队列有更具体要求，以该要求为准。
- [review_evidence.zip](review_evidence.zip)：`review_ledger.json` 含全部逐文件理由、SHA、函数/表达式定位、不变量、验证入口、原始分组审查和范围清单；另含本轮编译失败原始日志。
- [verification.json](verification.json)：报告完整性、静态核验、本轮未执行事项及新文档对后续打包的影响。

为控制报告体积，证据 ZIP 使用 Python 标准库支持的 LZMA。下列命令只解开审查数据到临时目录，不执行工程入口：

```bash
python3 -m zipfile -e docs/readability/overnight_20261112/review_evidence.zip /tmp/fc01-overnight-review
```

随后可在 VS Code 打开 `/tmp/fc01-overnight-review/review_ledger.json`，用“格式化文档”展开 JSON，搜索 `path` 或任务编号。

## 范围和评估口径

475 个已跟踪文件全部归类：312 份源码/工程配置候选，加 11 份冻结源码备份补审，再加 152 份文档、数据、发布产物等其余文件。323 份评估包含 26 份历史材料或旧入口，不应把它们全部称为当前生产代码。115 份 Python、138 份普通后缀 C++、36 份 Shell、11 份 YAML、5 份 MSG、3 份 CMake，以及 XML/TOML/格式配置等均纳入候选范围。冻结备份另列，不能重复计数。

3897 个 Git 忽略文件按来源归类为构建/消息生成物、哈希命名测试缓存、回放映射、解释器缓存或本地 IDE 元数据；没有将它们冒称为已完成语义审查。生成 `.inc` 的来源可追到 `vision_replay.py:341`。完整跟踪文件分类和忽略项计数在证据账本，忽略项原始列表的缓存位置及 SHA 也已记录。

逐文件评估结果：123 份“清楚”，127 份“需导读”，47 份“需分责”，26 份“历史”。“需分责”不等于必须拆函数；先理解状态和副作用，再决定是否值得提取。“清楚”只表示当前职责可以追踪，不表示没有未知逻辑问题。

判断依据是职责、输入输出、状态/资源所有者、物理量和时间域、分支原因、外部动作、测试夹具与算法的边界，不使用汉字数量作为完成标准。全部文件做当前字节/结构扫描；关键飞行链深入阅读，小模块直接阅读，大型测试及辅助段落结合当前入口/场景/夹具/断言端点和 SHA 绑定旧记录复核。没有声称本轮独立逐行重读所有历史代码，更没有声称数学证明全部状态安全。

已读取此前中文审查、Stage 2 结构覆盖/已知问题/验证、Stage 3 修复/结构债/配置/验证记录。Stage 3 的 probe 清理、适配器构造清理、视觉预算/水位、AsyncLog 修复不再列为未解决问题；[旧报告](../stage2_20261111/KNOWN_ISSUES_zh.md)是历史快照，当前状态以 [Stage 3 报告](../../reliability/stage3_20261112/REVIEW_zh.md)和本轮新增证据为准。

## 优先处理的阅读障碍

| 定位 | 本质困难 | 后续处理 |
|---|---|---|
| `attitude_altitude_core.hpp:1145`，`Core::step`，795 行 | 公共安全门、阶段转移、同帧继续下降和输出责任混在长协调函数；早返回前可能已经改变状态 | OVN-03：先列“进入条件→写哪些状态→本帧输出→退出条件”，再评估窄辅助函数 |
| `runtime.hpp:925`，`Runtime::step`，151 行；`:798` 的 `ack` | 本地请求、真正提交、ACK 和状态终态不是同一种成功；400 重用歧义跨任务保留 | OVN-04：事务词典及两个完整时间线，不能统一成一个 `success` |
| `fc01_system_node.cpp:367`，`SystemNode::tick`，360 行 | 接收危险事件、计划、三次边界采样、两个 sink、动作、审计和恢复同处 | OVN-05：先画时序，禁止把非原子发布包成单个布尔成功 |
| `rc_navigation.hpp:202`，`RcNavigationStart::step` | 开关边沿、真实 armed、Offboard、地面源推进各有时间约束 | OVN-03：解释 READY 只允许精确零流，恢复 READY 不是自动起飞 |
| `navigation_restore.py` 的 `activate/configure`；`standby.py` 的 `supervise` | 资源部分取得、外部命令部分成功、异常补偿与所有权水位 | OVN-07：为 `changed/started`、自建/复用 Agent 和句柄移交列事务表 |
| `navigation_service.py:338` 的配置工厂 | `lambda r,a: (check_controller(), load_profile(...))[1]` 利用元组求值隐藏执行先后 | OVN-07：后续可改具名窄函数，但须保留每次先核验二进制、再加载配置及异常顺序 |
| `test_rc_navigation.cpp`、`vision_test_support.hpp` 等 | `#define main` 后包含另一个 `.cpp` 复用夹具，虚拟时钟/默认健康值来自远处 | OVN-10：先建夹具依赖与场景索引，再考虑迁移；不能复制控制算法制造假测试 |
| MSG/配置/视觉适配链 | 米制尺度、NED/FRD、xyzw/wxyz、同戳健康、可信 epoch 和目标速度的权限边界 | OVN-09/11：字段词典、算例及配置来源图；不改类型、话题或视觉默认开关 |

这些行数的旧 Clang 结构指标仅在 SHA 与当前文件相同时复用；本轮纠正 Node 历史指标中四个定位偏移两行的问题，只修本轮引用位置，不改历史证据。行数/嵌套是定位线索，不是代码质量分数。

给初学者的共同解释还缺四张小表：`f/a/v` 是命名空间别名；`c/s/o` 通常表示配置/反馈或快照/输出，但每个作用域必须以声明为准；`q` 在测试里可能是整套夹具，在数学里可能是四元数；`now/received_at/stamp/sample_stamp` 分属单调秒、接收时间和源时间。不能批量改名或把这些时间变量合并。

阅读必须区分“观测事实→控制计划→实际本地提交→PX4 采用/物理结果”。tick 先消费中间危险事件，Runtime/Core 才形成计划；本轮计划包含姿态发布时，目标前、心跳前重新检查，完整 pair 后才允许关联动作及事务提交；独立动作仍执行各自最终权限门。报告后再检查恢复证据。健康末帧、一次 ACK 或 `publish=true` 都不能跳过这些层次。

## 缺陷、候选与历史输入缺口

**OVN-01 / P2 测试工具兼容缺陷，已最小复现。** `native/scripts/test.sh:13` 和 `native/scripts/r62_cpp_regressions.sh:13` 遍历新增日志测试，但未带 `--wrap=fcntl/write`；复制相同参数只编译该测试，退出 1，报未定义 `__real_fcntl/__real_write`。未执行脚本或二进制。`native/scripts/install.sh:18` 调用前一个入口，故这条原生安装前测试路径会受阻。正式 navigation 安装构建、CMake 和 `tests/run_native_cpp.py` 不因这个遗漏而失败。本阶段不修复；下一阶段独立修复并验证。

**OVN-08 / 待复现候选。** `velocity_command_diagnostic.py:693` 的 main 在 `node = DiagnosticNode()` 返回后才取得引用，构造中途失败的显式清理责任需故障注入验证。不能因为形态类似旧适配器问题就登记为已确认泄漏或飞行故障。最小注入方案已随 Python 审查记录保存。

**OVN-06 / P3 冗余条件候选。** Core 的 PoweredGround 分支在外层 `bench_arm_only_active_` 的 else 内再次检查该标志；后续先证明中间无写入及隐藏副作用，再决定是否等价清理，当前保护原样保留。

**OVN-12 / 历史输入缺失。** `native/scripts/r62_failure_replay.py` 固定引用的 `native/evidence/telemetry_20261011T034828Z.csv` 不在当前工程。只记录缺输入，不伪造样本，也不把这份历史回放计作当前可运行回归。

## 参数、验证及停止点

当前检查入库设置的纯配置派生值为：目标 1.5 m、模式档位 **1**、悬停/储备/最大推力 **0.50/0.04/0.80**、人工 RC 解锁、程序 Arm 关闭、视觉关闭。函数默认档位 6 与当前 `navigation_settings.json` 的 1 不是同一事实；均不证明飞控实机模式映射。`ground.yaml/flight.yaml/flight_test.yaml`、会话 profile 和 CLI 覆盖分层记录，缺失 YAML 键不擅自补成推测的默认值。

Stage 3 已完成的 534+41 次 Python 测试方法、80 个 C++ 程序、9 个独立 ASan/UBSan 程序、Humble 构建和隔离 ROS 验证作为**已有证据**保留，本轮没有重新执行这些全量测试。本轮实际执行：115 份 Python AST 解析、36 份 Shell `bash -n`、11 份 YAML 解析、27 份 Stage 3 修改源 SHA 核对及两份证据包 SHA/CRC 检查；另执行上述预期失败的最小链接复现。报告生成后再核验原 475 个跟踪文件未改。

TSan 仍是初始化失败，ARM64/真实视觉标定与延迟/PX4 采用仍无本轮验证。本轮未新增已确认飞行 P0/P1；可读性债与尚未证实的硬件现象不混为飞行缺陷。

本目录新增文档会进入打包器的递归 docs 范围，原 `FC01_RELEASE.json` 保持 Stage 3 历史身份，不能冒称它已覆盖本轮新报告。OVN-13/14 要在最后交付时刷新当前清单、核验原 8 MiB/512 文件/单文件 1 MiB 限制；不能为装下报告放宽保护。本阶段完成报告后立即停止，不执行后续六阶段的代码修改。

当前报告已使包体接近上限：保留旧清单时仅剩约 3 KiB；仅刷新哈希映射和身份字段的只读估算余量不到 2 KiB，尚未生成或验证新包。后续增加注释和文档前就应复查体积，不能等到最终阶段才发现超界；精确字节数见 `verification.json.package_assessment`。
