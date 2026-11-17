# 第七阶段：独立可读性复核

源码冻结点：`4c3c360941575049ebac78dbbf4ce58f098d93b6`。今晚基准：`5b86eeb4094ada009903c0140f66289cfe89411e`。

本子审查只读；只把清单、指标和报告写入仓库外缓存，没有修改源码、配置或历史证据，没有提交、推送、部署或启动任何控制程序。源码摘要复核未发现审查期间变化。

## 1. 清单与真实审查深度

当前 528 个跟踪文件全部登记路径和 SHA256。350 个源码/配置/显式元数据候选逐项列于 `final_readability_ledger.json` 和 `final_file_index.tsv`；其余 178 个文档、数据及其他文件也在 JSON 的完整跟踪清单中。

| 类别 | 数量 | 含义 |
|---|---:|---|
| 当前扫描器识别的自主源码/配置 | 316 | C++、Python、Shell、MSG、CMake、YAML与三份明确工具元数据 |
| 本次补充显式元数据 | 7 | 两份设置JSON、release/verification/baseline JSON、VERSION、.gitignore |
| 当前候选合计 | 323 | 是审查清单口径，不表示323份都已逐语句人工复核 |
| 冻结备份 | 10 | .bak/.before_/.original，保留原字节 |
| 独立历史反例 | 10 | c_independent_repro / c_s10_independent |
| 历史证据源码 | 1 | docs/evidence中的合成轨迹 |
| 生成安装包 | 6 | dist中的历史自解压包，不直接修改 |
| 全部候选 | 350 | 每项都有当前摘要和唯一分类 |

当前323候选中，本次完整人工阅读34份，人工检查关键流程13份，其余276份完成当前字节、职责入口、结构和复杂度索引，**没有完成独立逐分支人工复核**。27份历史/生成候选只检查来源类别和摘要。所有结论按真实深度记录；本次不把此前阶段的自评直接继承为“独立通过”。

当前候选语言分布：C++132、Python126、Shell36、YAML11、MSG5、CMake3、格式工具配置1、Python工具配置1、包XML1、额外元数据7。84份当前候选相对今晚基准有变化。

因此，不能把本轮说成“整个工程的每一行已经像中文教材一样完成复核”。逐文件清单没有遗漏，但人工语义复核仍分深度。`all_source_criteria_met=false` 是本次的真实结论。

## 2. 确认存在的可读性改善

1. `attitude_types.hpp`把动作许可、人工RC路线、参数候选和真实反馈分开，解释结构体默认值不表示状态已验证；默认档位6和实际配置覆盖也有明确说明。
2. `attitude_reference.hpp`解释了const引用、optional为空、值副本、可写引用、活动任务reset和yaw半角四元数。输入FRD→NED、wxyz顺序以及rad单位可直接从文件查到。
3. `vertical_controller.hpp`解释了`height=z0-z`、`speed=-vz`、外环速度前馈、速度PI、试算积分和真实提交、物理限幅/起飞临时上限/slew的区别。增益量纲及中间状态副作用与当前表达式一致。
4. `vertical_feedback_guard.hpp`明确`|vz-z_deriv|`只给诊断，不独立停机；位置与导数梯形积分仍有硬故障路径。微秒源采样和秒级窗口分开，重复采样不提供新证据。
5. `Runtime::step`和`Core::step`说明输出是计划，早返回不会回滚已写成员，候选状态不能提前完成命令事务，人工RC与程序Arm不混用。
6. `SystemNode::tick`按输入、请求、计划、最终发布、审计和重新准备分段；snapshot复用图计数不等于给旧数据续期。发布前重龄和非原子target→heartbeat顺序有解释。
7. `publication_boundary.hpp`解释模板转发引用、同步lambda、可空借用指针、首个sink已返回后的部分提交，以及裸throw不吞掉原异常。本地publish返回和PX4采用明确分开。
8. 五种MSG给出字段语义、单位、坐标轴、valid位、源时间与本机接收时间、估计状态与制导目标的区别。新VelocityCommand的FRD目标速度仍没有取得Runtime/PX4发布权。
9. 视觉转换代码解释ENU/NED、FLU/FRD、世界对齐、传感器安装旋转和杆臂`ω×r`；明确尚未完成协方差变换，NaN和未置有效位不会伪装成可信协方差。
10. probe/collector/adapter/diagnostic的关键清理路径说明谁拥有Node/Context/信号处理器，写证据失败也继续尝试后续释放，首错和次生错误不靠吞异常隐藏。
11. CMake/XML解释消息生成、类型支持、运行依赖、可搬迁RPATH和Python相邻依赖；正式入口、旧入口、离线工具及可能操作服务的脚本在注释上有所区分。

这些结论来自实际代码与相邻解释的核对，不来自汉字计数。语法基础说明已明显增强，但读懂整条飞行链仍需要结合各阶段导读和数据字典。

## 3. 仍未完成的结构债

以下是当前源码实际行范围，包含注释和空行；跨度不是圈复杂度，也不是要求强行拆分的理由。

| 文件/函数 | 起止行 | 跨度 | 后续最有价值的工作 |
|---|---:|---:|---|
| attitude_altitude_core.hpp / Core::step | 1160–2005 | 846 | 独立维护阶段/早返回路径表；拆纯计算前必须保留状态修改与浮点顺序差分 |
| fc01_system_node.cpp / tick | 398–764 | 367 | 保留sink顺序的分段职责审查，避免抽出后改变异常和提交证据 |
| fc01_system_node.cpp / state | 1955–2112 | 158 | 按既有JSON字段组做纯序列化辅助函数，保持字节和null语义 |
| runtime.hpp / Runtime::step | 1015–1181 | 167 | 事务候选/完成/新请求的执行顺序仍需导读 |
| rc_navigation.hpp / RcNavigationStart::step | 208–317 | 110 | 为READY、人工解锁、导航边沿建立一致的证据时间图 |
| launcher.py / run_console | 556–726 | 171 | 输入、IPC心跳、子进程及终止责任仍混在协调器 |
| launcher.py / main | 851–985 | 135 | 报告、预检、启动、异常退出可继续做有差分保护的窄拆分 |
| navigation_restore.py / activate | 493–624 | 132 | 激活/证据/快照/失败恢复仍需顺序图 |
| navigation_restore.py / configure | 670–788 | 119 | 配置生成与托管安装副作用需要进一步明确分段 |
| standby.py / monitor | 539–637 | 99 | 状态、writer、IPC和进程结束仍耦合 |
| standby.py / supervise | 854–973 | 120 | Agent/native的责任分界与线程异常仍值得专项审查 |
| native/tools/fcctl.py / run | 265–480 | 216 | 命令行模式/交互分支多，基础较弱者仍需调用图 |
| vision_odometry_adapter.py / _adapter_node_type | 582–829 | 248 | 包含嵌套类与多方法，不是单个算法；依赖注入便于测试但初读较难 |
| velocity_command_diagnostic.py / main | 828–1017 | 190 | 包含嵌套类和生命周期，适合增主线图，不应直接合并到生产控制链 |

精确Python AST和指定C++函数范围保存在`long_function_metrics.json`。其余长测试/工具也逐文件列于ledger，不限于上表。

25份当前候选仍通过`#define main`后包含另一`.cpp`复用测试夹具。已有中文说明解释这种做法，但这仍是结构耦合；后续提取公共支持头需要覆盖全部依赖，不能只把include文字搬走。

大测试文件如`tests/stage2_python_trace.py`、`test_navigation_diagnostics.py`、`test_navigation_install.py`、`test_velocity_command_diagnostic.py`、`test_vision_state_validation.py`和真实ROS smoke脚本，本次只建立结构/入口索引，不能声称每个fixture和断言均已独立复核。

## 4. 注释与布局中的剩余问题

- `ground.yaml`对时钟、profile和若干限幅连续出现含义相近的两组中文解释；`receive_events.hpp`、`publication_boundary.hpp`也有相邻重复说明。它们并非完全无信息的机械注释，但会增加初学者翻阅成本，适合后续仅注释压缩。
- `unix_server.hpp`头部“短步执行”不能读成整轮请求数/耗时已被限制；Stage 6 的IPC工作预算候选仍隔离，当前说明宜在后续补足限制边界。
- `DiscreteEventLatch`容量限制事件个数，底层是`std::deque`；不能据“容量固定”推断为静态预分配、无动态分配或push不会抛异常。后续可补函数异常合同。
- `diagnostic.yaml`、`flight_test.yaml`各有两次同值`visual_navigation_enabled: false`。当前仍为关闭，重复键会让维护者误判覆盖关系；本阶段没有为了整理而改加载语义。
- Shell/安装器模板中仍存在受历史字节/载荷等价检查保护的紧凑表达式，例如`common.sh`单行if和安装模板内嵌Python。没有把C++排版改进扩展成“所有语言所有代码都没有压缩”。
- 既有扫描器未自动识别10份冻结后缀备份与上述7份元数据；本次在清单补齐，没把它们报成文件缺失，也没把历史文件列成已中文化。
- 本次去掉注释/字符串后扫描C++行外层分号，未发现多业务语句压在同一行的候选。此结果仅限当前C++和该规则，不能证明表达式简单，也不能覆盖Shell模板。

## 5. 风险和结论边界

本次可读性样本没有新增确认的P0/P1控制缺陷结论，不代表它们不存在；本子审查也没有执行测试，实际回归结果应以父任务的最新源码验证为准。

已知安全/生命周期候选仍需保留隔离，不能为了本报告可读性而合入生产行为修改。尤其地面证据计时、每轮IPC工作预算、旧启动清理竞态、standby线程启动/结束责任，不能被已有注释或大量测试通过遮盖。

未真实验证ARM64调度、PX4实际采用、相机/SLAM质量。视觉时钟同步确认字段仍是上游声明，话题同名和单发布者不等于认证，协方差变换仍缺失。这些边界在当前注释/导读中已有明确表述。

本次交付的是可追踪的当前清单和分深度独立核对：重要边界比今晚开始时更易读；长函数、跨测试夹具、重复解释及未完成的逐分支独立复核真实保留，不把“索引完备”写成“所有语句完工”。
