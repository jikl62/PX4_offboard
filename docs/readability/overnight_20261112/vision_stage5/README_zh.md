# 队列 5/7：ROS 2、视觉接口与配置完整性交付

本轮从干净的 **`68b3221ac297699e1dfed8fd773358bb26789207`** 开始，在 `readability/fc01-chinese-20261111` 本地开发。此前 Stage 3 及队列 1–4 的历史保留；没有修改 main、推送、部署或运行真实动力程序。源码交付点为 **`15310f8776d182e668cc846f97ff23c5753995da`**，文档另作本地提交。

完成了 Odometry、VisionHealth、VisionState、NavigationCommand、VelocityCommand 的接线审查、准确中文说明、两处离线配置修复和模拟验证。**真实工程仍为 `visual_navigation_enabled=false`；VelocityCommand 仍只进入只读诊断器。** [交接与阅读路线](VISION_HANDOFF_zh.md) 面向初学者解释实际值/目标值、NED/ENU、FRD/FLU、外参、杆臂、时钟、跟踪与 QoS。

## 独立配置修复

| 等级 / 提交 | 原问题及复现 | 修复与边界 |
|---|---|---|
| P3 / `1124daf076596a2c49a857cbaaa3c883b6bd0d0f` | JSON 的 `10**400` 可在 `math.isfinite` 溢出，使候选导出 CLI 返回 1 和 traceback；原新 2 方法出现 1 FAIL、32 个子测试 ERROR。 | 只捕获 OverflowError 并转成既有 ControlError，保留原 cause。16 项数值参数、正负两种符号均覆盖，CLI 按原错误合同返回 2，在写输出之前拒绝。有限正常输入和原判断次序不变。 |
| P2 / `44632a18f5ef403ca09c5a84844f1fc52925595e` | 显式启用的候选可把 VisionState 和 NavigationCommand 设成同一话题并成功导出，但生产 Node 已会拒绝。原新 3 方法有 2 FAIL。 | 在导出前镜像生产 Node 的同话题拒绝；**只作用于启用候选**。关闭视觉且两个未用话题相同的原兼容路径保留。已有输出文件不会因这两种配置拒绝而被覆盖。 |

最后的配置测试为 **6/6**，含 9 组正常输入/既有拒绝的基线差分，比较规范 JSON 轨迹文本逐字相同及返回对象身份；JSON 键排序，不比较字典插入顺序，不归一化新增错误边界。新增拒绝条件独立提交，没有混入注释提交。没有把 `--enable-vision` 删除：它仍只导出明确指定的候选，不回写真实配置或激活服务。

`apply_settings` 原地修改内存，失败不回滚先前字段；调用者须丢弃失败候选。候选导出使用 `write_text`，成功时可覆盖显式输出且不是原子写入。这些原合同已准确注释，本轮不把它们宣称成完整事务。

## 中文注释与协议保护

3 份 C++ 头文件的 **9,684 个完整原始 token 相同**；5 份 MSG 的 **99 条字段/常量/默认声明相同**。新增说明包括：实际 z 的正负例、完整旋转与限幅的区别、质量评分与协方差、绝对点与位移、严格时间配对、控制权限边界。

3 份视觉 Python 生产文件完整 AST（含原 docstrings）相同；`vision_config.py` 相对本轮起点、`vision_settings.py` 的注释部分相对两项修复点也保持完整 AST。补充说明明确：实际 ROS 使用 GuardedOdometryAdapter，不会自动运行离线 Validator；`time_sync_valid` 是声明而非实测时钟同步；发布者数量不证明 QoS 匹配或来源可信。

CMake、package.xml、环境 Shell 只增中文说明，可执行内容未变。**54 份受保护文件原字节相同**，包括 Core、Runtime、SystemNode、RC、垂直控制/反馈、服务/恢复、视觉设置、全部原生配置和现有 dist 文件。没有改推力、1.5 m 平台、解锁、RC 接管、Kill、Failsafe 或最终发布时序。

逐文件理由、保留文件、源码 SHA 见 [changed_files.json](changed_files.json) 与 [review_ledger.json](review_ledger.json)。已经充分说明的资源预算、配置和历史接口测试没有为制造修改而重写。

## 本轮真实验证

| 验证 | 结果 | 证据与限度 |
|---|---|---|
| 根 Python 全量 | **612 方法：609 通过、3 ERROR；0 FAIL、0 跳过** | `root_final.log/json`。相对 Stage 4 新增 8 方法；原有测试未删除或弱化。 |
| 视觉相关离线 Python | **148/148** | 转换、配对、质量、资源、生命周期和诊断；属于全量的重叠子集，不累计成额外覆盖。 |
| 新接口合同 | 4 组、39 条 C++ 断言通过 | 有效位缺失、坏源时间、撤销后健康尾帧、精确 TTL/源年龄边界；ASan/UBSan 通过，另有独立严格编译复核。 |
| C++ 全量主机入口 | **81 个当前顶层 test 程序，聚合测试通过** | 根全量运行真实主机 C++ runner，包含新合同；使用假 sink/合成状态，不是实机动力验证。受影响的 5 程序也有独立严格编译运行日志。 |
| 新 Humble 构建 | 成功，46.938 s | 全新 cache 下 colcon Release 构建/安装本包；链接真实 px4_msgs 下层。生产控制器只编译，未执行。239 份 native 构建输入前后及最终一致。 |
| 五消息生成/类型支持/CDR | **5/5** | ament、生成类、类型支持来自指定新安装前缀；MSG 与源码字节相同，字段/常量/默认值匹配。非默认数组、中文字符串、`uint64 > 2**53` 两次解码精确。只加载类型，不 init ROS。 |
| 实际状态适配器通信 | **7 项通过**，退出 0 | localhost 域 220，运行新安装脚本；转换、来源冲突、资源撤销、新鲜恢复和 SIGINT。发布端点只有状态及 ROS 元数据，无 `/fmu/in`。 |
| 目标速度诊断器通信 | **11 项通过**，退出 0 | localhost 域 221，运行新安装脚本；真实类型、BE/RELIABLE 接线、重复/乱序/失效/来源/reset、超时和 SIGINT；无飞控控制发布者。 |
| 静态与格式 | **126 份 Python 编译、9 份变动 Python Black、C++ clang-format、36 Shell 语法、diff 检查通过** | 编译语法不等于逐行人工审查所有历史文件。Shell 未被当作控制入口执行。 |

两个通信域均在发合成数据前核验空图，并在退出后核验清理。4 份已安装视觉 Python 和 5 份 MSG 与最终源码字节相同。初始 smoke 清单中的新 CDR 工具随后因自身断言修正而变化，单独保留每次 SHA；**其余生产工具和通信驱动未变**，不冒称整个初始清单始终相同。

新 CDR 工具的首两次真实运行失败也已保留：第一版误把 Humble 元数据 `SLOT_TYPES` 当作协议常量；第二版错误要求 CDR 对齐填充字节稳定。受控分配器 A5/5A 重放证实五类各 20 次差异均只在精确填充位置。最终仅豁免这些 CDR1 对齐位置，仍逐字段两次解码及比较全部非填充字节；uint64 位破坏、float64 符号破坏和额外协议常量均正确拒绝。未因此更改消息或生产代码。

### 三项全量 ERROR 仍是发行阻塞

三个测试均在打包源文件枚举报 `Curated source payload exceeds extractor bounds`：

1. `test_navigation_install.BundleTests.test_candidate_directory_and_development_experiment_exclusion`
2. `test_navigation_install.BundleTests.test_exact_self_extraction_repeat_and_conflict`
3. `test_vertical_diagnostic_bundle.VerticalDiagnosticBundleTests.test_current_checkout_sources_and_local_edit_are_exact`

[Stage 4 报告](../python_stage4/README_zh.md) 已独立完整导出该阶段起点并复现三项错误：当时源码 9,096,500 字节已超 8,388,608 上限。本轮保留原预算、测试、发布身份与旧包；没有把当前源码说成已有合格新安装包。后续需单独解决打包输入/证据归档策略，再完整验收。这一项是 **P2 已确认的发行阻塞**。

本轮没有重新跑不受影响的 native Python 41 项、TSan、ARM64 构建或实机性能测试；不将历史结果充作本轮新通过。Humble 构建与有限模拟不能证明任意 DDS 故障、负载或飞行状态安全。

## 未解决项、候选与下一步

| 分类 | 证据和后续任务 |
|---|---|
| P2 候选 / 低质量中间帧 | 直接 API 按 `.9 → .1 → .9` 连续写 Inbox、最后才调用 Controller，可让低于 `min_quality=.5` 的中间帧未变成撤销水位；若 tick 实际看到 `.1`，则按原逻辑请求 LAND。生产视觉回调增加事件计数，活动输出时停止后续 receive pass 并进入 tick；已核实本机 Humble 16.0.19 默认单线程/普通 DDS 路径每批对同订阅只取一条消息，存在具体调度缓解（源码和库控制流证据已保存）。**未证明此 API 反例能绕过生产调度，不称为已确认现网飞行缺陷**。启用视觉或更换 executor 前必须单独验证接线可达性及阈值撤销策略，本轮不改安全行为。 |
| P3 / 同值重复配置 | diagnostic/flight_test 中仍有重复的 `visual_navigation_enabled: false`；含义未冲突，但存在可读性债。保留 YAML 原字节，后续独立清理。 |
| 已知接口能力边界 | 适配器未变换协方差，输出 NaN、位不置位；离线 Validator 不自动成为 ROS 防线；身份预算不限制 DDS 先期分配；graph count 不是认证。当前不以猜测数据补齐这些能力。 |
| 真实视觉接入条件 | 缺真实 Topic/频率、外参/世界对齐/尺度证据、时钟同步和延迟记录、quality 定义、reset/session 规则、真实录包。见交接文档清单。 |
| 板上验证 | 已具备新源码、生成检查和离线测试入口；仍须无动力 ARM64 构建及 CPU/内存/延迟测试，未声称通过 Orange Pi 验证。 |

本轮没有新确认的 P0/P1。配置修复不改变飞行闭环；候选保存在证据包独立源码/轨迹中，未提升为生产补丁。当前可继续离线协作与无动力构建准备，**不满足启用真实视觉导航或交付新安装包的验收条件**。

## 可复核交付

[verification.json](verification.json) 汇总命令、实际计数、提交与边界；[evidence.zip](evidence.zip) 保存原失败、最终日志、构建/安装身份、通信报告、差分、候选及交叉审查；[evidence_index.json](evidence_index.json) 逐项登记长度和 SHA256。不将构建二进制、安装树或大缓存提交为载荷。

本轮到本地源码与报告提交结束。没有 merge/rebase/强推，其他本地分支和远程跟踪引用保持；这里核验的是本地引用，没有查询远程实时状态。后续队列从最终实际 HEAD 接续，不自动启用视觉或执行尚未给出的下一阶段。
