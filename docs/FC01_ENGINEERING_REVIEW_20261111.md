# FC01 工程审查、修改和验证

本轮从实际提交 `89bbc9f`、干净工作区开始，在独立分支 `audit/fc01-engineering-20261111` 完成审查。附件列出的 `e825481` 是较早记录，未用它代替当前源码。建议中的问题逐项核对，不把推测写成故障，也不把主机测试写成实机验收。

当前定位仍是**可继续进行 ARM64 离线构建及拆桨验证的工程候选版**。原无视觉起飞流程继续使用 RC 人工解锁、导航授权、1.5 m 目标及原高度控制。新 `VelocityCommand` 仍只有诊断和离线预览用途。没有获得开启视觉飞行或进入新一轮动力试验的验证结论。

## 七项确认问题及处理

| 问题 | 实际复现及影响 | 最小修复与回归 |
| --- | --- | --- |
| 配对超时被新回调绕过 | `pair_timeout=.05`，两半消息间隔 .20 s，若定时器未先执行仍能完成健康配对 | `StampPairBuffer.odometry/health` 在新消息前先过期旧项；旧撤销先于新正常配对返回。检查两种顺序、边界相等、及时到达及同回调正常尾帧 |
| 适配器运行期参数合同不一致 | 来源修改显示成功但转换仍用旧来源；`use_sim_time` 则能实际切换时钟，违反 V1 启动时拒绝模拟时间的要求 | 全部启动配置及 `use_sim_time` 只读；服务层拒绝修改并保留原显示值。没有伪造热更新 |
| 适配器正常停止出现 traceback | SIGINT 已由 ROS 关闭上下文，finally 再次 shutdown 抛异常 | 区分正常中断和外部关闭，仅在上下文仍有效时 shutdown；真实 ROS 检查退出码 0、无 traceback |
| 旧安装缓存复用旧工具 | `navigation.sh install` 的构建键漏掉 Python 工具，完成证据只覆盖两个 ELF | 构建键加入 `native/tools`，完成证据覆盖整个安装树；工具、消息模块、typesupport 修改/删除/新增均使复用失效。旧目录保留，另选 retry 目录 |
| 独立 native 安装找不到工具 | 包复制到 `workspace/src` 后，CMake 的 `../../tools` 实际指向缺失目录 | 同步复制 `workspace/tools`，保留原 `release/tools`。隔离测试执行实际复制段和 CMake 安装声明，核对三个工具字节 |
| 连续任务审计历史持续增长 | 真实 Runtime 连续 RC 地面取消后，历史大小随任务数增长；不是已经确认的本次卡死原因 | 只保留最近 128 笔旧 Command400 事务；不清零全局提交次数、generation、ACK 因果水位或当前事务。生产候选配置连续 320 轮通过 |
| 视觉限幅掩盖算术溢出 | 两个有限速度经增益后每轴仍有限，二维模长却溢出；限幅乘零后错误激活零制动计划 | 限幅前检查原始目标和加速度及其二维范数。首次坏计划等待；已有运动仍走原降落锁存。恢复健康尾帧不能清除锁存 |

适配器修改在 [vision_odometry_adapter.py](../native/tools/vision_odometry_adapter.py)，安装修改在 [navigation_install.py](../navigation_install.py) 和 [install.sh](../native/scripts/install.sh)。核心诊断历史修改在 [runtime.hpp](../native/src/jikl_fc01_system/include/jikl_fc01_system/runtime.hpp#L727)，视觉算术修复在 [vision_navigation.hpp](../native/src/jikl_fc01_system/include/jikl_fc01_system/vision_navigation.hpp#L192)。各项修改前失败、修改后通过的驱动及日志保存在本轮证据归档中。

历史缓存修复改变的是 `command400_history()` 的保留长度，不能称作“修复内存泄漏”：旧 vector 在 Runtime 销毁时本来就会释放。视觉算术修复会改变异常输入在显式启用视觉时的拒绝行为；当前 `visual_navigation_enabled=false` 下不进入该路径。两项修改分别提交，便于单独审阅和撤销。

## 源码如何变得容易读

五个 `.msg` 的 99 条声明补充了中文物理含义、单位、坐标、时间与有效位解释；删除注释后，类型、字段顺序、常量及默认值与原版逐行相同。没有创建第二个同名接口包。

CMake 用中文说明类型生成、线程、相对 RPATH 和 Python 安装布局；删除注释后所有可执行声明相同。安装器四个构建/复用函数展开单行多语句和复合判断，整个 Python 模块 AST 与功能修复后的版本完全一致。

原始 Odometry 转换器按“显式健康合同 → 坐标/姿态 → 杆臂及实际运动 → 有效位 → 配对与节点”组织阅读，关键数学保留原运算次序。结构整理与功能修复分开提交，使用原/新完整状态、事件和缓存时序差分验证。没有大规模重排原起飞 Core 或最终飞控发布函数。

中文注释重点解释判断依据、数据来源和安全边界；物理公式按成组运算解释，不用“定义变量、进入循环”逐行凑字数。原有 [ground.yaml](../native/config/ground.yaml) 的中文说明保留，其参数没有改动。自动起飞实际使用的候选配置仍是 [flight_test.yaml](../native/config/flight_test.yaml)。

## 控制链和架构结论

```text
原起飞：PX4反馈 + RC边沿 → Runtime/Core → 原高度/推力控制 → 最终重采样检查 → PX4
已有视觉运动：VisionState + NavigationCommand → 已有视觉外环 → 同一权限及发布边界
新速度通道：VelocityCommand → 独立诊断（未接入生产控制）
另有准备工具：显式实际状态 + 目标 → C++ 离线预览（与诊断节点尚未自动连通）
```

`Odometry` 表示实际状态，`VelocityCommand` 表示希望达到的速度；二者仍分离。已有 `NavigationCommand` 的 Hold/Cruise/Follow/Land 与新速度消息不是自动互通的两个入口。换视觉算法可以保留标准 Odometry 和显式 Health 合同，但时钟、真实尺度、外参、方向与质量仍须实际确认。

实际飞行控制仍需要 PX4 的快速 Z/Vz 反馈；视觉没有替代角速度内环，也没有注入 PX4 EKF。原 RC/Kill/Failsafe 和接管优先级没有放宽。最终 target/heartbeat 各 sink 前的当前上下文检查保留；软件无法把“本地已提交”提升成“PX4 已执行”的证据。

## 证据和测试怎样理解

审查先核对职责、入口和控制权，再检查具体异常分支、构造边界复现、修复、回归，最后由另一位审查者反向复核。实际检查范围和每项工具结果列在 [本轮验证证据](evidence/20261111_engineering_verification.json) 中，含源码 SHA、原始失败日志及最终结果。

基线完整 Python discover 已真实运行 418 项通过；它只是起点，没有因此认定代码无缺陷。最终结果如下；不同轮次、专项重跑和完整测试中的相同用例不重复计入总数。

| 实际检查 | 最终结果 | 原始记录（本轮缓存目录内） |
| --- | --- | --- |
| 主工程 Python discover | 426 项通过，44.910 s | `root/python_full_verified.log` |
| native Python discover | 41 项通过，28.860 s；与上一行合计 467 项 | `root/native_python_final.log` |
| C++ 全量离线程序 | 73 个程序通过 | `root/native_full.log` |
| 真实 Humble 构建 | 干净构建及最终增量构建通过，安装源码与当前文件逐字节相同 | `root/build_clean.log`、`root/build_final.log`、`root/installed_final.json` |
| 安装后 ROS 通信 | 适配器 5 项、速度诊断 11 项通过；仅 localhost 隔离域，无 `/fmu/in` 发布者 | `root/ros_vision_installed.json`、`root/ros_velocity_installed.json` |
| 连续任务与 ACK 隔离 | 原生产候选参数 320 轮，历史上限 128，generation=320，自动 Arm/Offboard 均为 0 | `control/history_fix_verification.json` |
| ASan/UBSan | 事务历史、视觉外环、视觉 Runtime、原 RC 起飞、日志、接收事件/预算、最终发布及速度预览共 9 个程序通过 | `control/history_fix_verification.json`、`vision/navigation_numeric_fixes.json`、`root/additional_sanitizers.json` |
| 重构等价性 | 安装器整个 AST 相同；适配器正规化 AST 相同，62,592 次状态/异常差分及额外 58 次访问顺序检查通过 | `delivery/readability/`、`delivery/adapter_readability/`、`vision/adapter_readability_independent_review.json` |
| 视觉正常输入差分 | 300 个会话、3,000 步完整结果相同；异常溢出拒绝是单独的有意修复 | `vision/navigation_numeric_fixes.json` |

第一次最终 Python 全量运行过 426 项，其中 2 项集成测试失败，原始日志仍保留在 `root/python_full_final.log`。根因是测试固定等待 0.1 s 就发请求，负载下模拟状态尚未进入 Mission；原生产代码正确拒绝。人为延迟初始状态 0.35 s 可稳定复现，改为等待原 `Mission.observe()` 实际消费完整地面反馈后，同一复现通过。只修改测试；仍检查单次起飞、原控制超时、恢复 PING 和最终未解锁。没有通过放宽生产门槛让测试通过。

打包测试在功能修改尚未刷新 manifest 时曾有两项哈希拒绝，原始失败保留。最终单独检查安装、native 布局、源包和构建证据四个模块，47 项全部通过（2.397 s，`root/packaging_final.log`）。随后新增证据文件并刷新最终清单，再通过实际打包器的完整性入口逐文件核对。证据记录这两个快照的关系，不将文档哈希更新冒充重新编译。这些重跑不再叠加到 467 项。

320 轮连续任务是合成 PX4/RC 状态驱动真实 Runtime 的地面取消、Disarm、独立上锁状态和重新准备测试；不是 320 次真实飞行。测试核对原 1.5 m 和 `0.50/0.04/0.80` 参数、最近事务顺序、全部复用 ACK 结果不能误归属新事务，以及自动 Arm/Offboard 计数始终为零。ASan/UBSan/泄漏检测均有实际记录。

静态分析也有局限：cppcheck 2.7 在完整 ROS 翻译单元遇到 AST 解析错误；控制头文件的独立扫描完成。clang-tidy 首次因 GCC 默认 C++17 与 clang 默认语言级别不一致而失败，显式指定 C++17 后的结果另存。ThreadSanitizer 可编译，但运行报 `unexpected memory mapping`，因此没有线程竞态验证通过的结论。不得隐去这些失败或把工具退出码简单解释成安全证明。

## 仍然存在的验证或架构缺口

| 项目 | 已知代码事实 | 处置与下一步 |
| --- | --- | --- |
| 控制循环时延 | 接收额外 pass 有合作式预算；回调本身不可被这个预算抢占 | 需要 ARM64 实际负载/突发 DDS/磁盘压力测量分位数与最坏观察值；不宣称硬实时上界 |
| 可选 CSV | 节点的可选 CSV 写入会同步 flush；正式导航服务当前设 `csv_path=''`，其它启动入口可能开启 | 本轮未混入日志线程重设计。明确检查部署入口，后续若启用 CSV 应测磁盘阻塞；见节点 `write_csv()` |
| 视觉消息资源 | VisionState/NavigationCommand 字符串没有 ROS 有界长度声明，拷贝可能分配内存 | 当前视觉关闭时节点不创建这些订阅。启用前需确定大小预算及拒绝策略，不能只看 QoS 深度 |
| 协方差 | 纯 Python 状态校验有原始矩阵 PSD 检查；原适配器不发布有效协方差；生产 C++ 可选矩阵检查主要为有限值/对角线 | 未完成原点/杆臂/坐标完整 Jacobian 变换，不能写成“协方差已完整接入”；不准用零矩阵冒充高置信度 |
| 测量时钟 | PX4 源时钟检查与显式 ClockPolicy 已实现；视觉 Health 的 time_sync_valid 仍是发送方声明 | 需记录原测量时间、接收时间、时钟偏移及漂移，不能把声明当作已测时间同步 |
| PX4 异常输出响应 | 本地 partial publication、心跳上下文改变、stall 的拒绝路径有测试 | 仍需真实 PX4 参数、断流/接管/FailSafe 与同程 ULog；不能从 host sink 证明真实响应 |
| 旧安装包 | `dist` v5 是历史源包，342 个载荷文件；本轮基线源码清单已有 362 个 | 不使用旧包证明当前代码部署完成。本轮不覆盖旧包、不激活服务；当前源码与最终清单单独交付 |

原 `CONTROL_LOOP_STALL`、`PARTIAL_OR_FAILED_SETPOINT_PUBLICATION`、`HEARTBEAT_FINAL_CONTEXT_CHANGED` 的实机根因不因视觉测试通过而消失。垂直速度差仍为诊断；新鲜度、数据有效性和原飞行包络继续保留。过去出现推力不足、垂速异常或高度参考问题的真实根因，需要新的同步记录，不能在这轮只凭源码认定已经解决。

## 项目当前完成度

A 表示有本轮或明确保留的软件验证证据；B 表示已实现但实机/系统验证不足；C 表示当前工程未实现该能力；D 表示缺少外部资料或观测，尚无法判断。同一功能的软件和物理部分可以处于不同类别。

| 能力 | 分类与结论 | 对应依据 |
| --- | --- | --- |
| 无视觉 1.5 m 起飞 | B：流程实现并有软件回归；本轮没有实飞验收 | Core、RC 测试和未变的 flight_test.yaml |
| 高度控制、推力调度 | B/D：原闭环和起飞推力上限调度保留；真实悬停推力和动力余量未测 | VerticalController、bounded_liftoff_reserve 测试、候选参数 |
| 高度参考、垂速监测 | A/B：新样本、reset、诊断和重建有回归；传感器误差及真实地面待验 | ground reference、vertical_feedback_guard、RC 恢复测试 |
| 连续任务恢复 | A（软件）：320 次生产配置地面取消与重建；真实连续飞行仍 B | test_runtime_history_capacity.cpp |
| 控制循环稳定性 | B/D：软件拒绝过期输出；Orange Pi 实际抖动与极端负载仍未知 | receive_schedule、publication_boundary、ARM64 压力数据缺失 |
| 视觉 Odometry 与坐标转换 | A（合成数据/ROS）：转换、配对、原始来源数量和退出可验；真实输入仍 D | adapter 单元、ROS smoke、前后差分 |
| ToF、光流 | C/D：没有新的 FC01 直接驱动；可考虑由 PX4 接收和融合，但未验证现有接线/参数 | 本仓库订阅的仍是 PX4 输出状态；厂商资料不是实机证据 |
| VIO/SLAM | C/D：此仓库没有视觉算法本体；视觉同学的尺度、IMU 融合和跟踪性能未知 | 接口已定义，实际视觉代码/录制未交付 |
| 水平速度/位置闭环 | B：已有 NavigationCommand 外环及理想回放；新 VelocityCommand 生产适配仍 C | vision_navigation、visual_runtime；隔离 velocity preview |
| 时间同步 | B/C/D：PX4 检查已实现；视觉端同步机制和真实误差未确认 | source_clock、ClockPolicy、Health 声明 |
| 命令仲裁与视觉失效 | B：已有权限/撤销/降落锁存有软件回归；新速度通道仍隔离 | Runtime、Inbox、最终发布边界 |
| 避障与目标规划 | C：本轮没有完整避障、地图规划或可验的上层制导算法 | VelocityCommand 只是目标接口 |
| Agent/DDS | B/D：生命周期和来源门有软件测试；实际带宽、丢包与恢复时延待验 | service/transport 测试、硬件录制缺失 |
| Orange Pi 性能与消息兼容 | A（x86 类型/构建）；ARM64 性能、与实际 PX4 固件消息组合仍 D | 真实 Humble 构建/通信；没有 ARM64 测试结果 |
| 日志及诊断 | A/B：有界报告、来源、时间、部署身份和异常证据；真实 PX4 执行仍需 ULog | report、collect、diagnostic、build evidence |

## 交付和下一阶段

本轮修改按用途分开提交，最终文档与证据提交的哈希由交付时的 `git log` 和回复给出，不在自身文件内制造循环哈希。

| 类别 | 提交 | 文件范围 |
| --- | --- | --- |
| 配对/参数/退出修复 | `3fe0894` | `native/tools/vision_odometry_adapter.py`、`tests/test_vision_adapter.py`、`tests/ros_vision_smoke.py` |
| 安装完整性修复 | `28871d7` | `navigation_install.py`、`native/scripts/install.sh`、`tests/test_navigation_install.py`、`tests/test_native_install_layout.py` |
| 有界事务历史 | `1cd1fe9` | `runtime.hpp`、新增 `native/tests/test_runtime_history_capacity.cpp` |
| 注释/格式 | `0b5b4af` | 五个 `.msg`、包内 `CMakeLists.txt`、安装器四个函数 |
| 原生产配置长跑 | `214e6be` | `test_runtime_history_capacity.cpp` |
| 视觉数值修复 | `d66f7f7` | `vision_navigation.hpp`、`test_vision_navigation.cpp` |
| 适配器结构整理 | `888522f` | `vision_odometry_adapter.py` |
| 精确接口注释 | `8327a0b` | `NavigationCommand.msg` |
| 测试启动竞态 | `5251e45` | `tests/test_integration.py` |

中文注释的实际统计是：五个消息共 99 条声明均有中文说明；适配器 88 行中文注释、6 个说明字符串；安装器 19 行中文注释。CMake 有效声明不变。统计用于说明改造范围，不表示整个工程所有语句均已注释或每个未改文件都完成逐行审计。最终证据保存了独立审查者实际完整阅读的文件、关联片段和工具覆盖范围。

具体硬件、数据、视觉同学交付物、未决因素和试验阶段见 [下一阶段准备清单](FC01_NEXT_STAGE_PREPARATION_20261111.md)。最终 Git 提交、修改文件清单及源码哈希以 Git 与最终证据/发布清单共同确定；清单中的 `source_commit` 依既有策略表示验证前基线，避免追逐自引用的最终提交哈希。

本轮没有推送、重新生成 `dist`、部署或启动生产控制器、Agent、串口、systemd、电机。下一阶段先在香橙派用当前源码完成离线构建和安装身份核对，再进行真实 PX4 拆桨记录；不能因本轮静态/合成测试而直接开启视觉动力闭环。
