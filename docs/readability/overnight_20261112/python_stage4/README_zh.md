# 队列 4/7：Python、服务管理与安装工具交付

本轮从干净的 `f46468c21cac677dec0a3b300eb779de2fca44fd` 开始，始终在 `readability/fc01-chinese-20261111` 开发。源码交付点是 **`3ce3bc254eb30a67d1d67734afa4f8cb11c8db9d`**。此前 Stage 3、核心中文化及接口整理的提交全部保留。

实际审查了根目录 22 份 Python 和 `native/tools` 17 份 Python：14 份修改，25 份保留原字节并登记理由。另修改/新增 6 份测试文件，共新增 48 个测试方法。所有 123 份当前 Python 均做编译语法检查；这不等于逐行深入人工审查了每份历史测试。[逐文件账本](review_ledger.json) 区分了这些范围，[修改清单](changed_files.json) 给出全部 20 份源码/测试的原因与 SHA256。

没有改变运行入口、命令行合同、消息字段、Topic、源配置、控制器 C++ 或原有权限。`navigation_service.py`、`navigation_restore.py`、`standby.py` 在上一阶段已充分解释，本轮保留它们的源码，只复查关键路径与差分，不为增加修改量重复拆分。初学者可从 [Python 阅读路线](PYTHON_READING_zh.md) 开始。

## 独立功能修复

| 等级 / 提交 | 原问题与复现 | 最小修复与验证 |
|---|---|---|
| P2 / `d66b9057073b4fea82baefa7cf259ef2a9c7b12f` | `atomic_json` / `build_release` 独占创建临时项失败后，仍删除此前已经存在的 `.tmp-PID` / `.install-new-PID`。普通文件和软链四种碰撞下，原 5 方法出现 1 FAIL、3 ERROR。 | 只有本次创建成功才进入清理；碰撞仍拒绝，不覆盖、不重试。5 项归属回归和原 13 项构建证据测试通过。不是对抗任意同用户路径替换的 inode 租约协议。 |
| P3 / `8abc77b11dee5c754272b79618da455e482aba15` | `make_socket_path` 拒绝过长路径时，留下刚分配的空目录；没有创建 Socket，却累积会话目录。 | 只 `rmdir` 本次空目录；旧会话保留。若清理也失败，原 `ControlError` 仍可见并带次生错误。3 方法含 10 次重复拒绝和正常所有权移交。 |
| P2 / `9d0d4bccb15b0ad08eb77ffcee086bfc27c32e02` | 报告从 `status.json` 间接选择的运行入口仅按词法归属 home，`..` 或父目录软链可带出 home。原新测试能够收到 home 外来源。 | 保留原词法检查及展示路径，追加 `resolve()` 后的实际归属检查。8 方法覆盖外跳拒绝、home 内别名、显式入口兼容与错误传播。不是消除了后续目录替换的 TOCTOU。 |
| P2 / `7ebe9263e547325b9794bc32c5fba371e007cfa3` | 被动 `feedback_age_probe` 的 executor 清理失败可遮住订阅原错并跳过 Node、Context、文件；重复 init 失败还会关闭调用方已活动的默认 Context。原故障轨迹保留于证据包。 | 明确 Context 所有权，将取样缓存/时钟初始化纳入保护区；逐项尝试移除节点、停 executor、销毁节点、关闭自有 Context、关闭输出。保留原错误对象及原 cause/context；没有业务错误时抛首个清理错误。18 项生命周期故障注入通过。 |
| P3 / `79c53cb6cd1006ae24da01e8daef1587d06fa025` | 本地 JSON 的 `10**400` 进入 `math.isfinite` 可抛 `OverflowError`，使被动诊断退出。没有证据表明它曾导致真实飞行故障。 | `number` 将这种值标为未知；有限原类型保持，输入不修改。新增 3 方法。 |
| P3 / `c666d7797c216a60b8c2bc031ec77ae1aa89fadc` | 上项修复后，快照 `source_timing` 的超大正负源戳仍可在年龄算术处溢出。 | 既有 `read_controller_status` 错误边界只增加 `OverflowError`，按原约定返回部分诊断及明确 `error`。新增 1 方法含四种组合。`available=True` 只是读到快照根对象，不代表数据有效或飞行已授权。 |

本轮没有新确认的 P0/P1。上表的分级描述已复现的应用代码影响，不把诊断错误推测成飞行事故，也不把有限测试解释为全部时序的安全证明。

## 中文说明与结构整理

`3ce3bc2` 独立保存注释与等价重构，不混入上述功能修复：

- `Evidence._summary` 替代三处相同字典，仍在三个原写入点重新构造；保留嵌套对象引用、裁剪列表清空/更新、首错和写盘顺序。
- `_native_build_commands` 单独生成 configure/build/install 三组 argv，保留完整表达式、取值及执行顺序；路径中的空格仍是单个参数，不转为 Shell 字符串。
- `_record_status_identity` 单独解析已读到的状态字节并填写内存索引；报告编排由 115 行减为 85 行，读盘、只读命令、ZIP 写入和异常时点保留。
- 为 Agent 自建/复用、锁、Socket、日志、运行配置、被动 ROS、状态新鲜度、工具退出与错误边界补中文合同。保留原 docstrings，避免运行时帮助文字随注释整理变化。

10 份仅注释生产文件的完整 AST 与起点相同；反馈探针的注释版完整 AST 与 `c666d77` 相同；三处 helper 分别机械内联后，整文件完整 AST 与其独立功能修复点相同。构建另有 8 组协调器差分及 6 组取值/错误顺序交叉审查；报告有 6 组真实归档成员差分；Launcher 比较正常/超预算归档、首错与引用关系。源码注释也经过另一代理独立复核，并修正了容易误读的 Context、首错和 `events.close()` 所有权说明。

## 真实验证结果

| 验证 | 结果 | 证据范围 |
|---|---|---|
| 根目录 Python 全量 | **604 方法：601 通过，3 ERROR；0 FAIL、0 跳过** | `root_final.log/json`，604 包含新增 48 方法；原有测试和断言均保留。 |
| Native Python 全量 | **41/41，通过，无跳过** | 最终代码冻结后重跑；首轮只有 Launcher 注释变化，首轮记录及原因也保留，不冒充同一字节快照。 |
| Root 最终针对性 | **71/71，通过，无跳过** | 新资源/差分、反馈和原 Launcher；它们属于上述全量的重叠子集，不累计成额外覆盖。 |
| 与本轮起点差分 | **9/9 轨迹字节相同** | render/report/service/activation/standby/launcher/launcher_entry/fcctl/probe。两侧 probe 均已包含此前 Stage 3 的 disk-full 清理修复，不做新的结果归一化。 |
| 被动反馈真实 Humble | 正常 5 秒，退出 0；姿态 199、位置 198 次回调 | localhost 域 189；仅合成 `/fmu/out`，观察到探针只有 `/parameter_events` 发布者，未创建飞行控制发布者。600 ms 历史 sample 与 RMW 元数据正确保留，status 未改，stdout/JSONL 相同。 |
| 静态/格式 | 123 Python 编译、Black、`git diff --check` 通过 | 230 份 C++/消息/配置/Shell/dist 等受保护文件原字节一致；原服务/恢复/监督器原字节一致。 |
| 原有 C++/进程回归 | 根全量中的既有主机 C++ runner 与原生进程测试通过 | 是假 sink / 假进程 / 宿主程序，未启动生产控制节点或 Agent。保留静态 `systemd-analyze verify`，不执行真实 systemctl 管理。 |

Humble smoke 的精确源码 SHA 是 `0543d8d3d325bafe812142202209e215051ce3cd71c6cd107b383da09f25e7ef`。其后只增加本地快照溢出捕获及等价注释；最终 `main`、ROS 取样、序列化和释放路径 AST 保持该 smoke 点的执行逻辑。最终版本的异常捕获另经针对性和全量测试；**不声称最后一字节版本又重复运行了 ROS smoke**。本轮没有重跑 CMake 全量、ASan/UBSan、TSan 或 ARM64，也没有把历史结果充作本轮新通过。

### 三项全量 ERROR 的解释

三个测试均在 `build_navigation_bundle.names()` 报 `Curated source payload exceeds extractor bounds`：

1. `test_navigation_install.BundleTests.test_candidate_directory_and_development_experiment_exclusion`
2. `test_navigation_install.BundleTests.test_exact_self_extraction_repeat_and_conflict`
3. `test_vertical_diagnostic_bundle.VerticalDiagnosticBundleTests.test_current_checkout_sources_and_local_edit_are_exact`

独立完整导出 **f46468c** 后，逐项重跑同样全部 ERROR；导出树测试前后原始条目及模式一致。该基线源文件已经 **9,096,500 字节**，尚不含新清单就超出 8,388,608 字节上限 **707,892 字节**；文件数含新清单为 457，未超 512，单文件未超 1 MiB。本轮新增注释、测试、证据还会增加体积。基线错误并不能免除当前发行阻塞：**没有产出本轮可部署安装包，也没有刷新旧发布身份来冒充已验收**。保留预算、原测试及历史报告，后续单独处理打包输入/证据策略。

## 保留的结构债与候选

| 分类 | 位置、事实与下一步 |
|---|---|
| P2 / 已有发行阻塞 | 上述包体积与旧发布清单；后续独立设计输入策略并完整验收。当前不适合作为整树安装包交付。 |
| 未复现候选，涉及托管安全 | `standby._close_supervised_attempt`、`native/tools/start.py`：部分清理自身抛错可跳过后项。先注入各失败点、确定活原生/Agent 的责任，不能靠强杀或任意退出解决。本轮保持行为。 |
| 未复现候选，普通工具 | `navigation_collect.sample_ros` 部分 init/destroy 失败的清理；`DeploymentLog.finish`、`navigation_install.boot` 的日志关闭；仍需独立受控复现。没有登记成已证明的永久泄漏。 |
| P3 / 文件与事务边界 | `vision_config` 输出可覆盖显式路径且可能部分写入；`vision_settings.apply_settings` 失败不回滚已改内存；旧 deploy 不是完整补偿事务。保留现合同，不把它们说成原子事务。 |
| P3 / 错误提示 | 历史 autostart 包缺 argv、缺 git 可执行程序、旧 autostart.status 畸形字段的提示还可改善；不影响本轮已验证的正常入口。 |
| P3 / 长协调函数 | Launcher `run_console/main`、restore `activate/configure`、standby `supervise/monitor` 仍较长；执行顺序承载托管责任。后续按差分逐项分责，不为减少行数强拆。 |
| 测试导读 | 新 6 份测试有场景/替身/断言中文说明；既有历史测试本轮以完整回归和索引为主，未再次逐条改注释。 |
| 硬件与资源限度 | capture 后裁剪不是采集期内存硬限；路径归属检查不是完整竞态防护。x86_64 和正常 ROS 通信不证明板上负载、任意 OOM/磁盘错误或真实 PX4 采用。 |

人工 RC 解锁、无视觉 1.5 m 起飞、高度闭环、推力参数、正常降落、安全再准备、RC 接管、Kill 和 PX4 Failsafe 的相关代码/配置保持本轮起点字节。Python 不获得新的飞行权限，视觉仍关闭。没有推送、部署、改真实服务、打开串口、运行 Agent/生产控制器、解锁或操作电机。

## 可复核记录与交接

[verification.json](verification.json) 记录命令、计数、源码提交、保护范围与剩余错误；[evidence_index.json](evidence_index.json) 逐项给出归档原始文件长度/SHA；[evidence.zip](evidence.zip) 收入完整失败/最终日志、差分 JSON、归属复现、原/新轨迹、静态校验脚本和交叉审查。完整 Git 导出及构建缓存留在开发机 cache，未作为安装载荷或大型缓存提交。

源码提交后的工作区曾核验干净。报告另作本地提交，不在文档中自引用其自身哈希；没有新增 merge/rebase/强推，其他本地分支及远程跟踪引用保持。这里对 main 的检查是本地引用核验，没有查询网络或宣称重新验证远程实时状态。下一队列任务从本轮最终实际 HEAD 接续；本轮到报告及本地提交结束，不自动执行尚未给出的任务。
