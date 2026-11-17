# FC01 阶段3：真实ROS入口全流程验证与候选交付

2026-11-16，分支 `readability/fc01-chinese-20261111`。

**阶段2候选通过本机真实ROS软件集成验证；本轮没有修改生产控制源码。** 最终10个回放场景、1个简化闭环模型场景通过，另有50Hz零推力DDS测试和阶段2针对性回归。本轮未接香橙派/PX4，未部署、解锁或动力试飞。该结论不能代替真实飞行收敛与PX4失联测试。

## 候选、来源和差异

- 阶段3开始：`8767608f79a56e346d5654228770f948841fe0e3`。
- 生产功能候选仍是阶段2 `e8652ad686c52502ea877fd23b5805d6b64e0da6`，离地期限修复为 `63fde0293e61de4d85252f8bd19cf3c684e37067`。修复原理/旧失败新成功证据见 [阶段2报告](../20261116_stage2/STAGE2_REPORT_zh.md)。
- 阶段3测试候选提交：`ce931551a6f6bc39e2b50a5377d70608882650f7`。后续证据文档提交不改变其控制源码/测试字节，最终仓库提交用 `git log -1 --format='%H %s'` 查询。
- 本轮文件：`tests/ros_stage3_lifecycle.py`（真实ROS生命周期/故障夹具）、`tests/fixtures/stage3_runtime_20261114.yaml`（已采集runtime独立副本）、本目录的执行器/报告/证据。没有改 `native/src`、运行YAML、服务入口、推力/RC/PX4保护。[差异边界](evidence/SOURCE_BOUNDARY.json)。
- 原未跟踪的Stage1取证、MicoVIO测试和tools完整保留，未混入本阶段提交。没有main修改、merge/rebase/force push、远程推送或部署。

实跑ELF：`/home/jikl62/.cache/fc01-stage2-20261116/ros/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node`，x86-64 Release，SHA256：

```text
c14dbd9548217df28ceaf46ddc5de57b94a30e8b49dd9af9582668353defc0c0
```

复用阶段2已成功构建产物而非重新编译相同源码。[独立来源复核](evidence/BUILD_AUDIT_zh.md)核对47项输入摘要、36项实际工作区编译依赖、37个安装头及5个消息定义。每次场景启动核对生产输入与ELF，结束再核对输入未变。全部最终结果绑定同一harness SHA `8e6a54ba16d5796b71071f98a9bc91bded8abef18a41f3f7321fa18033ae6096`。没有把旧ELF与新源码仅凭HEAD文字硬绑。

## 实际验证环境与入口

Ubuntu x86-64、ROS2 Humble、FastDDS、原生C++ `fc01_system_node --ros-args --params-file ...`。使用原 `/fmu/out`、`/fmu/in` 话题、真实px4_msgs类型支持、真实DDS订阅回调和原IPC STATUS/LAND。合成peer提供PX4状态/反馈/RC/ACK，接收真正节点发出的目标/心跳/命令。固定domain191和localhost，启动拒绝已有其他节点；没有Agent、UART、PX4固件、动力执行器。**这是生产节点正常ROS入口的软件集成测试，不是PX4 SITL**；没有验证systemd→Python Supervisor→Agent完整服务栈。

夹具来自11月14日实际runtime，72个参数保持；只覆写本地socket、两个配置路径和测试入口身份。人工RC、导航档1、目标1.5m、视觉false，以及 .50/.04/.12/.80/.40 推力参数均保留；未放宽保护或超时。

## 最终11个场景

[逐场景完整结果](evidence/SCENARIOS.json)包含指标、阶段、事件和每项验收门；[独立复核](evidence/INDEPENDENT_REVIEW_zh.md)重新读取原始DDS数据并确认所有target/heartbeat时间戳集合相同、正常退出0。

| 场景 | 实际输入/观察 | 结果与范围 |
| --- | --- | --- |
| normal / replay | 安全地面z=+2；RC解锁边沿→新armed→导航开关→新Offboard；平滑高度输入 | 完成Ramp→独立离地→Ascend→HeightHold→LAND→接地→REQUEST_DISARM→SAFE_DISARMED→退出模式→新地面证明→第二次READY |
| normal / plant | 实际DDS推力驱动单轴动力学 | 完成相同全流程；下降超10s经过RECOVERY_DESCENT，详见下段 |
| range_unavailable | h<0.3m时range无效，z/vz始终有效；超过0.3m恢复 | 不阻断地面准备、离地、定高、降落、上锁和再次READY；真实节点观察到不可用→趋势恢复 |
| feedback_stale | 定高后停止local发布 | 约0.603s后观察DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE；恢复消息后保持锁存并停止双流 |
| quat_reset | 定高时quat reset计数+1，q保持有效 | 约0.105s观察ACTIVE_QUATERNION_RESET；不删除参考变化保护 |
| vertical_reset | 定高时z reset计数+1 | 约0.104s观察ACTIVE_VERTICAL_ESTIMATOR_RESET；不在armed重建z0 |
| offboard_loss | PX4状态从Offboard变STAB，开关不变 | 约0.104s锁存PX4_LEFT_OFFBOARD，不自动夺权 |
| offboard_flag | 仍报Offboard但flags声明心跳丢失 | 约0.104s锁存OFFBOARD_HEARTBEAT_LOST；不冒充真实链路丢包时延 |
| rc_takeover | 移杆但仍armed+Offboard→0.6s后模拟PX4模式退出 | 先受控DESCENT并持续发布；真正离开后锁存静默，总观察约0.757s含人为等待 |
| kill | Kill开关ON | 约0.105s观察KILL_OR_TERMINATION并锁存静默；不代表真实电机停转时延 |
| velocity_warning | z和z_deriv积分一致，仅vz加0.3m/s | 持续告警后再观察2s，保持HeightHold/feedback healthy与DDS输出，未因速度差单独切Safety |

故障观察时延是主机注入到IPC读到Safety，含消息周期与采样，不能等于飞控反应时间。7个故障均在恢复输入后继续armed+Offboard，检查首因不变、输出静默；排空窗同样拒绝Safety观察后生成的新戳输出。reset尾帧把counter恢复0用于对抗性锁存验证，**不是建议真实估计器回退计数器**。

正常降落特意将400 ACK延迟150ms，并在ACK-only后仍保持armed至少0.5s；节点不能仅凭ACK终结。之后独立推进DISARMED与接地样本，确认SAFE_DISARMED；原有上锁后176/STAB释放命令才获准。禁止自动Arm和进入Offboard命令；本轮正常接收的176是**地面退出Offboard**，并非改变起飞权限。模式/开关释放会短暂Safety(OPERATOR_TAKEOVER_LATCHED)，经原有新地面证明恢复第二次READY，继续等待新人工解锁，未自动起飞。

## 高度、定高和心跳指标

稳定窗定义沿用配置：高度误差≤0.15m、向上速度绝对值≤0.15m/s，持续2s。“稳定窗进入时间”是合格连续窗的起点，相对Ramp开始；HeightHold时间还包含控制器自身判据。定高误差仅统计进入HeightHold后约2s。

| 指标 | 正常平滑回放 | 单轴闭环模型 |
| --- | ---: | ---: |
| 目标 | 1.5m | 1.5m |
| 正超调 | 0m（输入预设） | 0.138944m |
| 稳定窗进入/Ramp | 7.527s（输入预设） | 7.931s |
| 进入HeightHold/Ramp | 9.673s | 10.123s |
| 定高最大绝对误差 | 0m（输入预设） | 0.138944m |
| 定高RMS误差 | 0m（输入预设） | 0.126575m |
| 最长心跳接收间隔，含窗口两端 | 51.018ms | 51.915ms |
| 最长目标接收间隔，含窗口两端 | 51.229ms | 52.125ms |
| 水平位移 | 0，固定输入 | 0，无横向模型 |

11场景最长心跳接收间隔51.915ms。窗口从新Offboard事实到Disarm命令（包括下降），故障场景至注入前；故障有意停发及第二轮重新预发送不混入周期抖动。目标/心跳严格递增、内部消息按同stamp配对；原始记录支持完整集合相同。速度告警后2s另由独立复核算得双流最大间隔约51ms，当前通用脚本指标仍截至告警注入；此限制明确保留，见独立报告。

模型采用 `a=9.80665*(thrust/0.5-1)-1.4*v`、上正积分、地面夹紧。无姿态、电机滞后、风、地效、噪声、真实DDS链路延迟；hover与模型恰好匹配。它仅验证此模型下的闭环与状态路径。**水平漂移真实值未测，不能把表内0写为抗漂移性能。** 回放不由输出驱动，其零超调/误差不证明控制性能。

模型LAND后约10.02s高度仍0.259m，触发原DESCENT_TIMEOUT_CONTROLLED_RECOVERY；约13.68s到LANDED，约16.64s发Disarm。未改下降时限、增益或保护以让模型“好看”。正常回放未经过恢复下降。该模型现象是下一阶段真实下降时间验证事项，不能据此认定真实机体必然失败，也不能声称全闭环模型完成了无超时普通下降。定高观察只有2s，未验证长时间漂移。

## 回归、失败与修复

本轮**未确认生产代码回归，生产修复数为0**。修正的是新测试夹具和验证结论，受影响场景在最终版本全部重跑：

- 原话题remap与生产所有权图查询不一致→保留原话题并隔离DDS域。图拒绝是正确保护。
- numpy float无法JSON序列化→转换内置float。
- 夹具把armed地面一律at_rest=false→地面静止按物理事实独立模拟，使接地Disarm证据成立。
- 收尾回放继续维持空中高度/在途旧包触发错误断言→明确单独cleanup地面输入和在途包排空，正常任务断言保留，异常收尾不吞错误。
- 初版ACK和DISARMED耦合、故障drain遗漏新输出、reset尾帧仍有故障、心跳漏两端及配对→按[独立初审](evidence/HARNESS_REVIEW_INITIAL_zh.md)补强。
- 分开确认上锁后看到了原176/STAB退出，原测试白名单错误拒绝→只允许已上锁接地且已有400之后的176(param1=1,param2=7)，禁止Arm/进Offboard保持。

[全部尝试清单](evidence/ATTEMPTS.json)含29份结果，6次早期FAIL完整保留，不能混作最终11次PASS。原失败/原始日志在early_attempts.tar.gz。最初ROS环境在nounset下source失败，后改用set-eo pipefail；新命令已修正。没有把夹具失败说成生产缺陷。

重新执行既有针对性回归：[完整命令/退出码/前后源码摘要](evidence/regression/results.json)。**40个步骤**包含编译，不能称为40项独立测试：13个C++程序；旧失败/新成功同一复现；4个ASan+UBSan程序；1个Python接收适配器unittest；10个行为差分unittest。全部符合预期，原样保留`repro-baseline`退出1，运行期间源码摘要一致。差分仍对3677b9d核对旧控制/wire字段，新增只读诊断按白名单处理，故障修复的有意变化单独验证。

额外现有真实ROS零推力冒烟：[50Hz结果](evidence/zero-thrust-50hz.json)。domain187、所有动作门关闭、5s观测；最大循环53.024ms、stall=0，local/attitude最大RMW到callback约21.7ms，无增长，丢反馈后约0.602sSafety；12对精确零目标/心跳、VehicleCommand=0、正常退出0。两个测试用不同域，最终模型晚于此测试，指标环境见原始记录。不将5s短窗扩写为高负载长期保证。

Python语法、Shell语法和git diff --check通过；未重跑无关全库。TSan、ARM64、真实PX4 SITL、实际Supervisor服务生命周期、硬件EKF/测距/飞行均未执行，不声称通过。

## Position对照与待现场验证

[Position档案复核](evidence/POSITION_LOG_AUDIT_zh.md)和清单：361路径、61保留路径、30独立内容，28硬件/2SITL；硬件档案没有Position(nav_state2)片段，唯一Position在旧SITL失败流程，不作稳定参考。用户现有MTF直连PX4 v1.16、Position实飞稳定仍作为现场事实保留，本地缺日志不能否定，也不能补造可比指标。

尚缺真实稳定Position ULog及候选版本同次ULog/香橙派日志。ARM64构建/负载、串口Agent链路、PX4实收心跳、COM_OF_LOSS_T退出、range有效性和估计reset时序、离地/地效、真实到高超调/长时间定高/XY漂移、下降10s窗口、多轮任务服务恢复均待现场。没有证据放开ACTIVE_QUATERNION_RESET或HEARTBEAT_FINAL_CONTEXT_CHANGED；部分发布仍保持原保护。拆桨未解锁检查不能验证空中动力学。

未发现新的已确认P0/P1回归。现场reset→停心跳→No offboard signal因果仍属**P1待证事项**，需同次双端日志；模型下降超原窗口属**P2验证关注项**，非已确认实机缺陷；真实Position对比与水平漂移缺测属**P2证据缺口**。这些限制不应被本机PASS掩盖。

## 交付、复跑和回退

候选源码/本地提交已保存，运行控制码保持阶段2。完整[复跑、拆桨未解锁现场核对及回退命令](VALIDATION_COMMANDS_zh.md)。本轮没有制作新板端发行、没有覆盖旧安装和FC01_RELEASE身份；现有20261112.1包不能冒充阶段2候选。下一步板端验证前需独立发行+ARM64身份核验，现场启动/切服务由人员执行。

完整ROS原始反馈/IPC/DDS/日志压缩包、差分trace和早期失败包在evidence；[归档哈希](evidence/ARCHIVES.json)记录文件数/大小，已逐成员读回校验。整体SHA256SUMS在本目录。安装旧目录/服务未动，无需因本轮本机测试回滚设备；源码可导出阶段2提交或逐提交审查revert，不执行破坏性reset，不清用户改动。
