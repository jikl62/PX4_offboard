# FC01 阶段 2：已确认缺陷修复与阶段 3 交接

日期：2026-11-16。工程 `/home/jikl62/FC01_OrangePi_Navigation_20261028`，分支 `readability/fc01-chinese-20261111`。

本轮修复了 **P2：离地运动证据超期后仍可被迟到检测器使用**，补齐通用排队故障首因及 PX4 测距趋势诊断。没有证据支持放开姿态 reset 或心跳最终上下文保护。现有融合 z/vz、地面 z0、人工 RC、推力和发布架构继续使用。

## 1. 基线、提交与差异边界

- 阶段 1 / 本轮开始完整 HEAD：`3677b9dd1f35e092e43501d26164c6263545c947`。
- 独立功能修复：`63fde0293e61de4d85252f8bd19cf3c684e37067`。
- 独立诊断与验证：`e8652ad686c52502ea877fd23b5805d6b64e0da6`，也是本轮最终生产源码提交。
- 本目录的后续文档提交只归档报告、命令、补丁及验证证据；最终仓库 HEAD 可用 `git log -1 --format='%H %s'` 查询。报告不自填自己的尚未生成的提交哈希。

[两个可单独复核的补丁](patches/)包含本轮生产修改和测试。需要撤销离地修复时可单独 `git revert 63fde0293e61de4d85252f8bd19cf3c684e37067`；需要撤销诊断时可单独 `git revert e8652ad686c52502ea877fd23b5805d6b64e0da6`。本轮没有执行撤销、rebase、merge、强推、推送或部署。原有 Stage 1 取证目录、MicoVIO 工具/测试等未跟踪内容继续保留，未混入本轮提交。

提交时发现本机未配置 Git 作者，首次提交因此失败，未产生提交；随后对单次命令沿用最近 5 个现有提交中的同一 jikl62 / GitHub noreply 身份，未修改全局 Git 设置。

## 2. 已复现并修复的问题

### P2：过期上升片段被迟到检测器错误配对

`TakeoffEvidence::observe` 原来仅在 **新 local 源戳** 分支检查最后真实上升时间 `motion_at_`。当持续静止 local 使缓存保持新鲜、最后上升即将超过 `land_timeout=2.5s` 时，检测器可以在两条 local 之间到达。此时 local 年龄仅 .3s，小于 .5s TTL，但旧运动年龄已 2.55s；原最终交集仍使用 rise_confirmed=true。

同一个 [Runtime 复现程序](repro/expired_rise.cpp)分别链接基线与修复源码，实际结果：

| 版本 | 退出码 | 最终状态 |
| --- | --- | --- |
| 基线 3677b9d | 1（预期复现失败） | rise=1、detector=1、confirmed=1、ASCEND |
| 修复版 | 0 | rise=0、detector=1、confirmed=0、THRUST_RAMP，继续原有 Ramp 托管输出 |

修复仅在最终证据交集前，每次调用复核主机域运动期限。新 local 的源域期限检查保留。边界为 **恰好 2.5s 可配对、超过 1µs 拒绝**；没有调整 dwell、阈值、目标、积分、slew、推力或实际离地条件。有效期限内的 detector 迟到仍通过原回归。

这是明确的控制阶段变化：**修复用例不再从 Ramp 错误进入 Ascend**。不能把这一变化说成完全行为等价，也不能据合成复现认定它就是此前现场失败根因。最新报告的瞬时故障载荷仍缺失。

### P2 可观测性：通用排队原因无法区分触发项

原 `RC_QUEUED_SAFETY_CONDITION` 将 10 类条件合并，阶段 1 已证明 failsafe 和摇杆事件能得到相同文本。现保留原 reason 和拒绝规则，另记真正锁存 Safety 时的首个 mask、事件种类、主机单调秒及各来源源戳。mask 可同时多位置位，不伪装成唯一来源推断；后续好/坏事件不覆写首因。Core 重建新任务会清除此任务诊断。

独立审查发现初版会把启动期临时 preflight=false 保存为后续真正任务首因，已修复：在原 reject 完成后，仅 Phase::Safety 保存。独立失败复现从 mask16/旧戳改为 mask2/当前戳；新增回归包括“启动临时拒绝→健康 READY→人工 RC 解锁→真正 failsafe”。初审与复审失败/通过证据均保留。

### P3 观测补充：有效测距趋势

节点旁观同一 `VehicleLocalPosition` 中的 `dist_bottom`、variance、reset 和 RANGE 位；不增加订阅/视觉依赖。要求 z 有效、range 有效、有限非负值、RANGE 位、原始样本推进、连续戳/reset；日志读取时检查同步时钟、信封及 raw TTL。失效输出 unavailable，gap/reset 不跨旧基线计算。

输出最近完成的至少 200ms 区间：`fused_rise=anchor_z-z` 与 `range_rise=distance-anchor_distance`、同向布尔值及区间长度。`range_sample_stamp` 为最新样本，完成区间的差分会保留到下次区间，不代表每条消息都重新完成差分。没有引入硬一致性阈值或安全许可。

**dist_bottom 是 PX4 地形/底面距离估计，不是已认证 MTF-02P 原始读数，也不保证独立于 EKF z。** RANGE 位说明参与估计的来源类型，不认证设备身份。FLOW-only、无效、NaN、超期均不能显示有效测距；不可用不阻断无视觉起飞。真实传感器质量与独立性仍待同次 ULog 证明。

## 3. 原有闭环与保护核验

既有代码已经满足下列要求，本轮通过测试核验，没有重复改写算法：

- `RcNavigationStart::observe_ground_reference/capture`：未解锁、接地、静止、反馈有效且源进展持续，完成窗口后建立任务 z0；解锁意图后冻结。
- `Core::rc_startup`：继承同一 z0，目标读取 `rc_target_height`；已采集 runtime 生效 1.5m，不硬编码绝对 local.z。
- `Core::step / height_feedback / VerticalController::step`：上升和定高统一 `height=z0-z`、上正速度 `-vz`。
- `advance_thrust_ramp`：原推力斜坡、提前反馈制动、联合离地后保持轨迹/积分/slew；到时/达到悬停推力不是离地证据。
- 到高仍需独立离地、非 landed、轨迹到目标、实际高度和速度在容差内连续稳定；经过目标但未收敛仍走原 Ascend 超时受监督下降。
- `abs(vz-z_deriv)` 单独持续偏差仍仅诊断；位置与导数积分不符、NaN、reset、源回退、失效等真实异常仍拒绝。
- 相同 q 但 reset counter 变化，尚不能证明参考连续；保留 `ACTIVE_QUATERNION_RESET`，后续普通好帧不能重新发布。目标已提交后的 `HEARTBEAT_FINAL_CONTEXT_CHANGED` 仍是部分失败，不能伪称完整未发送或补发旧心跳。

[PROTECTED_FILES.json](evidence/PROTECTED_FILES.json) 对 20 项配置/关键模块逐字节核验：运行 YAML、navigation_settings/service、vertical_controller、rc_navigation、attitude_reference、publication_boundary、vertical_feedback_guard 均与基线相同。人工 RC 解锁/触发、接管、Kill、推力 .50/.04/.12/.80/.40、PX4 失联保护及视觉关闭均保留。

## 4. 实际修改文件

| 文件 | 修改原因 |
| --- | --- |
| `native/src/jikl_fc01_system/include/jikl_fc01_system/takeoff_evidence.hpp` | 每次观察检查旧运动期限，4行最小修复 |
| `…/attitude_altitude_core.hpp` | 保留通用拒绝，同时记录真正锁存任务的首因诊断 |
| `…/range_trend_diagnostics.hpp` | 固定标量的只读测距趋势核对，无控制权 |
| `native/src/jikl_fc01_system/src/fc01_system_node.cpp` | 接入旁观诊断及新增 JSON 字段 |
| `native/tests/adapter_compile_check.py` | 夹具补齐真实 PX4 v1.16 dist_bottom 字段/常量 |
| `native/tests/test_receive_event_adapter.py` | 真实节点接收/序列化路径测试 RANGE、FLOW-only、失效/过期及原权限 |
| `native/tests/test_20261116_takeoff_expiry.cpp` | helper 期限精确边界和真实 Runtime 缺陷回归 |
| `native/tests/test_20261116_diagnostics.cpp` | 10 类触发、首因保持/启动恢复、range方向/reset/寿命/无效值 |
| `native/tests/test_20261116_height_contract.cpp` | NED 原点 -16/+32、目标 .75/1.5 下控制输出逐位相同，z0冻结、人工RC |
| `native/tests/stage2_20261116_captured_config.hpp` | 从已有板端runtime生成的64参数离线fixture，非运行配置 |
| `tests/test_20261116_stage2_equivalence.py` | 对3677b9d做严格控制/wire/原有状态差分，只允许18个明确新增诊断键 |
| 本调查目录 | 修复前后证据、执行器、交接命令、独立审查和可撤销补丁 |

## 5. 最终源码验证结果

[完整命令/退出码/源码SHA](evidence/regression/results.json)、[汇总日志](evidence/final-regression.log)、[ROS构建证据](evidence/ROS_BUILD.json)。全部测试对应最终生产源码；[提交内容复核](evidence/COMMITTED_SOURCE_CHECK.json)确认提交前测试字节与 e8652ad 一致，运行期间源码前后摘要相同。

- **13 个 C++ 测试程序**编译/运行均成功：3 个本轮用例，加离地证据、Ramp、height tracking、vertical guard、RC navigation、RC takeoff stage、时钟、522 发布组合、状态不变量、姿态/垂向/控制权测试。
- **4 个程序 ASan+UBSan**：本轮离地期限、本轮诊断、RC navigation、状态不变量；均退出0，无sanitizer诊断。使用 `-O1 -g -fno-omit-frame-pointer -no-pie -fsanitize=address,undefined -fno-sanitize-recover=all`。
- **真实 SystemNode 接收适配器夹具**：1个Python unittest 内多组 C++断言，退出0；覆盖速度差告警继续发布、真实位置残差/reset/NaN/回退拒绝、坏帧不能被好帧覆盖，另加测距诊断接线与TTL。
- **10 个差分 unittest**通过（包含3项差分过滤器自检和7项生产行为差分）；不能把内部组合数与程序数混加为“总测试数”。
- **ROS 2 Humble x86 Release**：1包完成，34.4s，源码/消息生成/类型支持/安装成功。参数检查器针对已采集runtime输出 `manual_arm_only=true target=1.5`；5种自定义消息的真实 Python type support 均加载成功，已安装新头与源头 SHA一致。没有启动控制器。
- `git diff --check` 通过。第一次新增诊断编译因const方法调用fixture非const get_clock失败，随后改成消费已有snapshot时钟而非另取时钟；最终实际Humble和所有夹具均通过。失败未被隐藏为成功。

[复现基线日志](evidence/regression/repro-baseline.log)的退出1是预期缺陷，不是最终回归失败；修复日志退出0。验证执行器分别校验编译成功和预期退出，避免把环境失败当成复现。

### 行为差分的准确范围

- Core trace **14,626行**、Runtime事务 **966行**、发布 **10,779行**逐字节一致。
- 522发布组合、256启动组合+25危险事件顺序+1024FIFO，原结果逐字节一致。
- 真实节点 **1,350帧恢复**和 **707帧人工RC起飞/1.5m定高/降落/新地面恢复**：原有JSON成员的数值字面量和顺序、全部目标/心跳/命令wire（含hexfloat浮点）完全一致。
- 新增18个只读顶层字段会使完整JSON变化；只按明确白名单去掉新增字段，不忽略旧字段或通配前缀。原始/过滤输出与hash均保存：[差分摘要](evidence/differential/)，[完整压缩trace](evidence/differential_traces.tar.gz)。
- 已确认过期运动用例是有意变化，单独做旧失败/新成功验证，不用“全流程一致”掩盖此修复。

有限确定性测试不等于证明所有状态安全。本轮未重新进行全工程中文化、完整Python全库回归、TSan、ARM64板端构建、新版真实DDS通信或实飞。未改变的模块旧验证不能冒充本轮重新验证。

## 6. 尚缺证据、风险和下一阶段

- **未确认新P0。P2离地期限和P2可观测性已修复**。初版新增诊断首因污染也已修复并独立复审，无剩余阻断发现。
- **P1待证审查事项**：现场姿态reset/停止心跳/PX4退出的同次因果缺完整双端日志；不是已确认误报，原保护保留。
- **P2证据缺口**：阶段1的最新版排队故障详细输入已轮转，旧报告不能靠本轮诊断反向补造；旧ULog中垂向残差和当前真实性能也不能混为同次根因。
- **P3测距限制**：目前旁观PX4 dist_bottom，不认证MTF原始数据及独立测量，现场range有效性/方差/融合来源/地形与姿态影响未实测。
- 未连接香橙派/PX4，ARM64运行、资源负载、DDS投递、实际飞行收敛与失联延迟仍待验证。控制器本地publish返回不能证明飞控收到。

[阶段3可直接执行命令](STAGE3_COMMANDS_zh.md)包括一键离线回归、ROS独立构建、板端只读取证、后续新版独立发行与无动力ARM64构建的入口及同次日志判据。不要覆盖旧发行目录或修改旧FC01_RELEASE清单冒充本轮已发布。本轮没有制作新部署包、推送GitHub、部署、解锁、启动电机或实飞。
