# 视觉导航独立审查（2026-10-31）

最终限定复核结论：本报告的 2 项 P1、1 项 P2 已全部关闭，新增原始 Odometry/Health 来源冲突保护也已核对。没有发现仍然成立的 Critical/Important 阻断项，可以交付默认关闭的竞赛视觉导航开发版本。该结论不批准硬件运行、安装或服务切换。下方最初 findings 保留为修复前历史证据，以本节最终状态为准。

范围：基线 `6b440ab`，工作分支 `feature/vision-navigation`，读取 `/tmp/fc01_vision_review_20261031.diff` 及工作区实际文件、设计稿和用户文档。未改项目文件，未启动 ROS graph、硬件、现生产 node/service/agent；仅在 `/tmp` 编译运行一个纯 host 定向复现。完整测试及真实 Humble 构建由主代理/其他既有代理负责，本审查未重复全量。


## 修复后独立复核（最终状态）

本轮严格限定于已报问题、上游来源保护、文档和安装 RPATH；未重复全suite，未启动生产 Node、ROS graph、硬件、service 或 agent。

| 审查项 | 最终状态与证据 |
| --- | --- |
| R1：first-sink TTL 窗口 | **关闭**。生产 Node 现调用 `publish_attitude_resampled`。initial 和 target 前检查的拒绝保留 Decision 返回，VisualInput 由 Node 请求原生下降；target 已调用后的策略拒绝抛异常，保留 partial-sink 硬路径。独立编译运行实际 `test_visual_runtime.cpp`，覆盖 249→251 ms 两个窗口及恢复下降，PASS。 |
| R2：Inbox 掩盖身份/epoch 变化 | **关闭**。身份变化和可信撤销先于 seq 过滤；本地 `revocation_generation` 同时约束 Controller `step/current`，保留 A→B→A 的证据且不阻断初次初始化后的正常启用。独立编译运行实际 `test_vision_navigation.cpp`，PASS。 |
| R3：default-off 拒绝收紧原包络 | **关闭**。视觉包络与原包络比较只在 enabled 时生效；独立运行高度2/倾角3/升降0.1的关闭兼容测试，PASS。已读原生配置回归及主代理通过结果。 |
| 原始 Odom/Health 多源被适配器隐藏 | **关闭该缺口**。`GuardedOdometryAdapter` 在 callback 和 timer 上接收两个真实 publisher count；0/多源清配对缓存并发出 mask=0 撤销，单源恢复必须新配对。独立运行4项来源/缓存/撤销定向测试，PASS。生产 Node 使用相同 helper；主代理报告隔离真实 ROS smoke 通过 ENU 转换、源冲突撤销、无 fmu publisher。 |
| 安装消息库解析 | **已核对**。CMake 给安装产物设置 `$ORIGIN;$ORIGIN/..`；只读 `readelf -d` 确认安装 `fc01_system_node` 和 `libjikl_fc01_system__rosidl_typesupport_cpp.so` 均含该 RUNPATH。真实最终 build/install 日志已到 `FC01_RC_NAVIGATION_CONFIG_VALID manual_arm_only=true target=0.5`；不从本报告推断生产环境启动状态。 |
| 组合与交付说明 | **缺口已补**。Runtime 增加视觉活动状态下 RC stick、mode-out、Kill 与 partial TTL 的组合检查；文档新增默认关闭设置、离线配置导出、SDK 构建/消息检查、完整 adapter 参数、合成 state/command/health 示例、原始唯一来源与假标定说明。 |

独立执行命令/结果：

- `g++ -std=c++17 -Wall -Wextra -Werror -pedantic -I native/src/jikl_fc01_system/include -I native/tests native/tests/test_visual_runtime.cpp -o /tmp/fc01_vision_review_runtime_fixed`，执行输出 `VISUAL_RUNTIME_CONTROL_PASS`。
- 相同严格编译选项编译 `native/tests/test_vision_navigation.cpp` 到 `/tmp/fc01_vision_review_inbox_fixed`，执行输出 `VISION_NAVIGATION_INBOX_CLOCKS_VALIDITY_XY_FEEDBACK_FOUR_AXES_FINAL_CURRENT_LANDING_LATCH_PASS`。
- Python 精确选择 default-off 保守包络 + raw source count/缓存清除/timer撤销/缺 metadata 5项测试：`Ran 5 tests ... OK`。
- 只读查看主代理 `/tmp/fc01_vision_final_suite_20261031.log`：`Ran 160 tests in 49.923s / OK`；完整suite不是本代理重跑。

这次限定修复未发现新增 Important 问题。下方 Declined to judge 中的设备性能、真实相机标定与生产状态限制仍适用；ROS 环境说明以主代理实际隔离 smoke 证据补充，不再把“未运行任何 ROS smoke”当作整体交付缺失。

## 修复前历史记录（以下 findings 均已关闭）

## Important / P1：第一个真正 sink 之前过期仍会硬中止并丢失原生下降

位置：`native/src/jikl_fc01_system/src/fc01_system_node.cpp:212-215,228-231`。

触发：视觉任务已经正常运动；外层 `publish_attitude_at_boundary` 的策略检查仍允许（例如状态年龄 249 ms），随后 target lambda 调用第二次 `boundary_context()` 的过程中跨过 250 ms TTL。lambda 尚未调用 `attitude_pub_->publish`，但任何 policy 拒绝都被转换成 `TARGET_FINAL_CONTEXT_CHANGED` 异常。catch 无条件设置 `boundary_fault_latched_` 并 `abort_attitude()`，使 Core 进入 Safety，永久停止这条控制流的发布。

用户效果：单纯视觉输入过期在尚未提交任一 sink 的窗口内会中断 Offboard 控制流，而不是保留 custody 进入原生受控下降。新增 `decision.reason == VisualInput` 恢复只处理外层初检拒绝，无法覆盖 lambda 内的这个真实窗口。

复现证据：`/tmp/fc01_vision_review_repro.cpp` 使用真实 Runtime、真实 `publish_attitude_at_boundary` 和与 node 一致的 lambda/catch 逻辑，无 ROS。运行 `/tmp/fc01_vision_review_repro` 输出：

```
FIRST_SINK_TTL initial_allowed=1 threw=1 sinks=0 phase=SAFETY
```

修复验收：在 target lambda 内也保留结构化拒绝原因和是否已尝试实际 publisher 的证据。只有纯 VisualInput 且任何 sink 均未开始时走 `request_visual_land`；PX4 health、Kill、RC/权限拒绝和 publisher throw/两 sink 间失效仍走原硬路径。必须覆盖“外层允许→target 内 TTL 拒绝”和“target 已提交→heartbeat 前 TTL 拒绝”两个独立测试，而不只测试直接调用 policy 过期。

## Important / P1：Inbox 的全局序号去重掩盖来源/epoch 变更

位置：`native/src/jikl_fc01_system/include/jikl_fc01_system/vision_navigation.hpp:271-284`。

触发：已接受并执行 `sequence=100, reset_counter=0, source_id=A`；来源重启后发送 `sequence=1, reset_counter=1, source_id=B`（或者同源 estimator reset 后序号重置）。Inbox 在比较 source/frame/reset 之前直接因 seq<=旧 seq 返回 false，控制器永远看不到新失效证据。旧样本仍被 `current()` 认可，直到旧 TTL 到期；如果旧来源随后恢复更高序号，变化证据可能完全消失。

用户效果：已知估计坐标/来源发生改变后，仍继续按旧任务坐标输出运动；与“来源/坐标/epoch 改变即锁存降落、不允许续控”的合同不符。状态和命令两个入口都有同样问题。直接对 Controller 的 source/reset 单测不能覆盖 ROS 实际先经过 Inbox 的路径。

复现证据：同一 `/tmp` 程序先通过 Inbox 接受 100，再向 Inbox 发送新的 source/epoch + seq=1，随后用 snapshot 调真实 Controller，输出：

```
RESET_SEQ_ROLLBACK accepted=0 active=1 land=0 current=1 stored_epoch=0
```

修复验收：完全相同来源/epoch 的重复旧消息可以不刷新 TTL，但已观察到的来源/坐标/epoch 切换应有独立撤销/锁存证据，不能被序号过滤掉。加通过 Inbox→Controller/Runtime 的 state/command 组合测试：来源切换+低 seq、epoch切换+低 seq、相同 seq 身份变化，以及完全相同重发不续 TTL。

## Important / P2：视觉关闭仍会拒绝合法的更保守原控制配置

位置：`vision_settings.py:38-47`；入口 `navigation_service.py:55-56`。

触发：保留原导航的合法较低包络，例如 `aa_max_height=2.0`；`navigation_profile()` 成功。新增 `load()` 默认 enabled=false 或没有配置文件时仍灌入 `visual_max_height=5.0`，并无条件检查视觉包络，导致 ControlError。同样影响 `aa_max_tilt_deg<5` 或原升降速度小于 0.15 的合法旧配置。

用户效果：用户根本没有启用视觉，原导航的配置生成/启动也被阻断。这是默认关闭路径的兼容性回归。当前默认基线 aa_max_height=5 恰好不触发，已有“参数值没改变”测试不足以证明关闭路径兼容。

定向 Python 复现输出：

```
DISABLED_REDUCED_ENVELOPE ControlError 视觉限幅不能扩大原倾角/高度包络
```

修复验收：视觉关闭时不要因为未使用的视觉候选包络拒绝合法原 profile；或明确将候选默认上限绑定到已有上限。保留启用时的严格包络约束。增加关闭/文件缺失、分别收紧高度/倾角/垂直速度的回归。

## 缺失验收/交付材料

1. 当前 7 组 Runtime 测试没有在已启用且正在执行视觉的情况下实际走 RC 接管、Kill、mode-out、partial sink 以及上面的 target lambda 窗口；旧通用测试能证明原机制，不能单独证明新增 visual 分支未改变组合行为。需要定向补充这些组合，尤其 first-sink vs partial-sink 的区别。
2. 用户文档说明了抽象参数和离线回放，但没有告诉集成人员 `vision_settings.json` 的字段/完整示例、如何用 `navigation_service prepare-config` 只生成并检查配置，也没有 `ros2 run ... vision_odometry_adapter.py` 的完整参数文件例子、VisionHealth 与 raw Odometry 配对最小发布例子、QoS 配对说明。实际代码已提供配置入口，文档仍不足以让不读实现的用户操作该临时 ROS 合同。补例子时明确只在隔离测试环境运行；真实外参保留待填，不用单位外参冒充设备标定。
3. 主代理已确认真实 Humble+FC01 SDK 的自定义消息、两个二进制 build/install 通过，日志为 `/tmp/fc01_vision_ros_build_20261031.log`；全量回归及最终汇总仍由主代理提供，本次未重复运行。

## 已核实的正面结果

- XY 真正由视觉位置→目标速度、视觉速度误差→加速度→四元数生成；不是仅建立 ROS 消息壳。
- 高度目标进入原 Core/VerticalController；候选参数不替换原垂直 PID，降低目标的 HeightHold 路径已有 max_descent 轨迹限速。
- Follow 四路分别有速度/yaw率/高度积分语义；中 throttle 为保持积分高度目标，非推力清零。
- 原始 Odom twist 按 child frame 转换；外参、世界对齐和杆臂显式配置。非零杆臂缺角速度时撤销速度 validity，未变换协方差保持无效。
- 最终边界在 visual_motion 下才放行精确绑定 plan 的受限倾角；默认关闭保持原 level guard。
- 正常过期从 Core 进入原生 Descent，正常双 sink 提交后冻结最后提交的 yaw；已存在相应 Runtime 测试。
- 没有发现识别/攻击接口，也没有发现回放器建立 /fmu/in publisher 的代码。

## Declined to judge

- 实机闭环稳定性、可飞性、VIO 精度/频率/时延、真实相机外参、PX4 航向对齐和动力学：无真实设备合同/数据，本审查只读与纯 host 定向复现，不能从理想植物推断。
- DDS 网络环境、设备 QoS、运行时来源 discovery：没有运行真实 ROS graph；生成接口/编译通过不能证明设备联通。
- 现生产 service/agent/飞控参数状态、安装切换是否安全、实机 RC 接管：明确不启动/修改生产组件，且不属于本次源码交付授权范围。
- 其他代理尚未最终完成的全suite结果：不重复运行，也不预先宣称通过。

最初重新交付条件现已满足：上述 3 个 findings 已修复并定向复核，新增组合测试和可操作 ROS 配置说明已补充；主代理全suite与真实构建证据见顶部最终复核。可以交付默认关闭的竞赛视觉导航开发版本，硬件验收仍单列。

补充审查状态：R1/R2/R3 修复后的独立复核已完成，最终状态见顶部表格。
