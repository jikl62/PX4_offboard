# 后续六阶段可执行清单

所有动作以本轮 `5b86eeb4094ada009903c0140f66289cfe89411e` 为初始基准；每阶段先检查实际 HEAD/未提交修改，前一阶段有新提交时采用其明确交付点，不回退 Stage 3。以下是处理建议，不替代队列中更具体的用户指令。本阶段仅建清单，不提前执行。

逐文件入口在 `file_index.tsv`，每项完整理由和当前 SHA 在证据包 `review_ledger.json.files`。`clear` 文件以保留和索引为主，不为增加修改数重复注释。任何发现的新逻辑缺陷先复现，再最小修复，独立于注释/等价重构提交。

## 阶段 2：先建立可信入口和初学者导读

**OVN-01 / P2，确认的测试入口缺陷，优先。**

- 目标：`native/scripts/test.sh:13`、`native/scripts/r62_cpp_regressions.sh:13`、`native/scripts/install.sh:18` 与 `native/tests/test_stage3_async_log_resources.cpp`。
- 复现输入和完整编译命令：证据账本 `confirmed_tool_defect_reproduction`；原 stderr 在 ZIP 的 `legacy_runner_repro.log`。原命令退出 1，不运行任何硬件程序。
- 后续修复：使两条 Shell 入口对这个测试使用与 CMake/Python runner 一致的专属链接参数；保留严格告警、断言及 glob 中全部当前测试。不要给生产二进制加入测试钩子，不要跳过日志测试。
- 验收：先用原命令复现失败，再用修复后的实际入口参数编译/执行该纯主机测试，核对故障注入确实触发；验证其他程序编译方式不变。必要时跑整个纯主机集合；不能直接执行 `install.sh` 或部署来验证测试参数。

**OVN-02 / P3，阅读顺序与术语。**

- 目标：生产 C++ 的别名/短名、`README_zh.md` 与 `README_NAVIGATION_zh.md` 的不同历史入口、当前正式导航链。
- 产物：配置→回调/事件→快照→Runtime→Core/RC→纯数值→最终 sink→事务确认→恢复的一页导读，以及四层结果、四种时间、进程/任务/样本寿命小词典。
- 验收：读者能从一个配置字段追到校验、状态条件、数值计算及最终输出；能说明 READY 为什么不等于解锁或起飞。保留公开名称、命令、故障字符串、Topic、JSON 键；先解释合法缩写，不批量改标识符。

## 阶段 3：控制时序与低风险结构处理

**OVN-03 / P3，Core、RC、数值和离散事件。**

- 目标：`Core::step`、`RcNavigationStart::step`、安全事件/恢复、`takeoff_evidence`、垂向控制和反馈守卫。
- 先做：阶段/进入条件/写状态/本帧输出/退出/下一任务重置表；用实际变量解释地面参考、raw 样本推进、双源离地、积分试算和饱和。
- 可抽取范围：独立纯计算或有明确输入/输出及副作用合同的窄阶段；不要求把 795 行机械切成固定长度。
- 禁止变化：首拒绝、早返回、同帧下降、浮点运算次序、保留制动/退积分、离地和地面证明、安全锁存。Vz 与 z_deriv 差继续只诊断。
- 验证：`test_stage3_control_equivalence.py`、`test_stage3_state_invariants.cpp`、RC 恢复/离地/垂向回归；注释与排版使用 token/AST/格式等价检查。实际改变行为须另立修复。

**OVN-04 / P3，Runtime 事务。**

- 目标：`Runtime::request/ack/step` 及 `ground_link`。
- 产物：requested/published/sent/accepted/state_verified 对照；176 快/负偏移候选和 400 重用各一条完整时间线；任务历史可淘汰与永久因果水位的区别。
- 处理：可以先命名纯分类/只读展示；不合并不同证据门，不因新任务清全局歧义。
- 验证：ACK lifecycle、M1 R3/R4、176 clock、320 轮 history capacity，以及 Runtime/节点精确差分。

**OVN-05 / P3，tick 与 publication。**

- 目标：`SystemNode::tick:367`、`observe_pending_safety:915`、`snapshot:1477`、最终边界/审计。
- 产物：逐观察点时序图，说明哪些事实来自计划、调用尝试、返回成功或后续 PX4 状态。
- 不可重排：逐条危险事件→计划→整周期期限；计划包含姿态发布时，目标前检查→target→心跳前检查→heartbeat→关联动作→提交确认。独立动作执行各自最终权限门；报告后的恢复另取当前证据。
- 验证：实际节点/动力链假 sink 差分、512 观察组合+10 异常、partial sinks、接收坏→好中间事件。不得把两个 sink 称为原子事务。

**OVN-06 / P3 候选，PoweredGround 重复 bench 判断。**

- 位置：`attitude_altitude_core.hpp:1687` 附近。
- 先查：外层 else 内到内层判断之间是否存在标志写入、回调、宏或其他可见副作用；对所有编译分支确认可达性。
- 若确为不可达冗余，单独做等价清理并检查 Core 轨迹及 bench/普通下降矩阵；否则保留并写清理由。禁止为了删“重复”而减少地面/托管保护。

## 阶段 4：Python 生命周期与外部事务

**OVN-07 / P3，正式导航/恢复和监督器。**

- 目标：`navigation_restore.py`、`navigation_deploy.py`、`navigation_install.py`、`navigation_service.py`、`navigation_probe.py`、`standby.py`、`launcher.py`、`native/tools/fcctl.py`。
- 产物：恢复 activate/configure/rollback 的动作与补偿表；changed/started 水位何时提前记；Agent 自建/复用、子进程、channel、泵、fd、锁的持有/移交/释放表。
- 优先窄改：具名配置工厂代替 tuple/lambda 副作用表达式；纯证据拼装与外部动作分离。write_json/atomic_json/write_status 先比较权限、fsync 和失败传播再考虑复用，不只凭函数形状做 DRY。
- 不变量：业务原错可见；原 cause/context 与次生错误保留；只释放自有 Context/Agent/句柄；signal handler 仍只请求停止；不能因 UI 退出遗弃原生托管责任。
- 验证：restore/deploy/activation、standby/launcher、Stage 2 Python 轨迹，以及 Stage 3 probe/adapter 生命周期测试。正常轨迹与故障路径分开比较，不把额外清理差异混进行为等价声明。

**OVN-08 / 待复现候选，速度诊断工具的部分构造清理。**

- 目标：`native/tools/velocity_command_diagnostic.py:693` 的 main / DiagnosticNode。
- 最小方案：在假 ROS 或隔离 localhost 测试中，于基类 Node 成功后、参数/订阅/定时器创建后分别抛错；记录创建/销毁/Context/异常顺序，再测试活动调用者 Context 和信号交错。
- 只有复现后才登记修复及严重性；不得从静态形态推断永久泄漏、默认视觉崩溃或实飞故障。若确认，按 Stage 3 的明确所有权合同最小修复，保留原始错误并独立提交。
- 验收：失败点逐个清理、原错误可见、正常重复退出/真实只读 ROS 回归；不改目标速度诊断的 `control_authority=false`。

## 阶段 5：视觉与消息合同的中文算例

**OVN-09 / P3。**

- 目标：五份 MSG、`vision_ros_adapter.hpp`、`vision_navigation.hpp`、`velocity_command_input.hpp`、适配器/状态验证器/资源预算与视觉设置工具。
- 产物：Odometry 自身状态和 VelocityCommand 目标分工表；NED/FRD 与 ROS/Hamilton 四元数算例；杆臂速度方向；reset/source/sequence/epoch 提交与拒绝水位图；协方差/PSD 的小矩阵例。
- 补注释：让身份字段直接指向“最多256 UTF-8字节、非256中文字符、拒绝而非截断”的共享合同；JSON 显示副本与有效身份不同，DDS 可能早于应用门分配。
- 不变量：消息包/字段/schema/Topic 不改；保留可信水位与双侧半包撤销；视觉默认 false；只读目标速度不取得飞行权限；不把未实现的 PX4 EKF 视觉注入写成已完成。
- 验证：ROS schema/安装身份、现有视觉/速度/预算回归与正常输入精确对照；若生命周期或接线有变再运行真实隔离 ROS。硬件尺度/时钟/标定缺口继续明确。

## 阶段 6：测试、Shell、CMake 与配置入口

**OVN-10 / P3，测试导读和夹具结构。**

- 目标：127 个“需导读”及47个“需分责”中对应测试条目，先处理 RC 主测试、ROS smoke、嵌入 C++ 的节点回放和共享夹具。
- 产物：主题→fixture→生产路径→观测层→现有测试入口索引；解释 `#define main`/包含 `.cpp`、假 ROS 类型、固定PID、虚拟时间、位掩码与场景编号。
- 先按场景导航，不移除断言，不放宽数值容差，不把虚拟时间当墙钟，不从缺少 assert 文本推断没有检查。真要提取夹具须编译所有包含者并复核整轨迹。

**OVN-11 / P3，入口和有效配置地图。**

- 目标：36 Shell、3 CMake、YAML/MSG/XML、`navigation_settings.json` 与 `vision_settings.json`。
- 产物：只读/编译/安装/启用/控制五级入口表；源 YAML→profile→设置/CLI覆盖→会话 YAML 图；`source` 与 `exec`/trap/退出码的含义。
- 区分：当前配置档位1、函数默认6、PX4模式映射；动作总门、自动Arm、Takeoff、视觉分别是不同权限。注释里不建议复制关闭保护的参数。
- 验证：bash -n、Shell执行载荷/heredoc等价、YAML类型/值、CMake token、MSG声明/安装合同；不执行部署脚本以检查语法。

**OVN-12 / P3，历史与缺输入工具。**

- 目标：索引26个 historical 条目、6个 dist 生成脚本、备份和记录数据的来源；说明 `r62_failure_replay.py` 的固定 CSV 缺失。
- 用中文旁注映射历史代号到当前正向测试，保留原始字节和失败解释；不把所有历史反例纳入格式化，也不创造假实测数据补输入。

## 阶段 7：当前身份、回归及交付停止点

**OVN-13 / P3，打包身份与体积。**

- 开始基准包：437 文件、8,250,853 字节，距8MiB仅137,755字节。本轮新报告进入docs递归范围，旧发布清单不会自动覆盖它们。
- 最后按实际当前源码刷新清单并验证逐文件哈希、sidecar、ZIP CRC、重建确定性及损坏拒绝；保留旧验证历史。不直接编辑 dist 嵌入载荷，不放宽8MiB/512/1MiB限制。
- 可压缩原始审查记录并保留可读索引，不删历史证据为新报告腾地方。超界则明确调整打包输入策略的独立方案，不冒称包已可用。

**OVN-14 / 验证与完整交付。**

- 对每项实际修改做针对性验证，再跑必要全量；分别列根 Python/native、C++独立程序、Sanitizer、真实隔离ROS、安装完整性，不重复累计同一程序。
- 注释/格式与有意修复分别提交。对控制结构改动使用此阶段起点与当前的确定性差分：阶段、权限、原因、浮点字段及sink顺序；旧基线f06715a用于承接Stage3证据，不覆盖今晚新提交。
- 报告真实失败；TSan若仍不能初始化就标未完成。x86_64、有限状态组合和主机模型均不代替ARM64、飞控采用或自由飞行验收。
- 保存新逐文件SHA、剩余债和独立提交，核对工作区/分支；不改main、不强推、不自动push、不部署、不解锁或起动电机。完成后停止等待下一次明确任务。
