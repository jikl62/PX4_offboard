# FC01 M1 0.4.4-offboard-flight-dev-r2 — Agent B 第二轮修复与风险报告

**状态**：AUTHORIZED OFFLINE DEV ONLY / NO-GO FOR FLIGHT。用户固定输入为 B r1 整包 SHA256 `61dcd8c9288d92c0cf3d6e02e2261172973853fec5db5ccd134adcca961839af`。C 的独立审查原始反例未被修改，保存在 `tests/c_independent_repro/`。本次不连接 FC01、没有实际 ROS publish、没有 Arm/Takeoff 或 PX4 参数写入。**本报告属于智能体 B 自审，不替代智能体 C 第二次独立安全评审**。

## C issue -> 代码改变 -> 证据状态

| 编号 | 修改位置与要点 | 本轮离线判断 |
|---|---|---|
| C-B01 | `publication_guard.hpp::attitude_publication_policy`：共同态度门控（包括 0 精确推力、准备和地面 release）校验 output 四元数、有限 yaw、相对水平参考角度；不再在 `!thrust_nonzero` 时跳过。模板 `publication_boundary.hpp` 双 sink 在非法 q 下均为 0。 | C04 GREEN，6 相位零推力姿态注入拒绝；仅离线 |
| C-B02 | `Core::descend` + `PoweredGround`：armed/landed/at_rest=false 的 LAND、disconnect、shutdown 走有界 `vertical_->ramp(0,dt)`，继续留守经审查的 Offboard setpoint；不因 `at_rest=false` 直接 Safety，不在 ground 未证明时 Disarm。 | C01 GREEN，at_rest=false 保留心跳，后续取得新的地面样本才 Disarm |
| C-B03 | `THRUST_RAMP` 超时分叉地面非确认/受控下降；**超时的同一 tick** 已计算有界减动力输出，避免默认 0 造成阶跃；依据推力、ground-detector 样本与 Status+ACK 进行有序退出。 | C02 GREEN，逐 tick slew 检测；旧“超时立即 Safety”断言更换为连续撤动力断言 |
| C-B04 | `Runtime::request(AUTHORITY_TAKEOVER)` 只调用 `request_operator_takeover`；若仍 armed+Offboard，保持下降/撤动力而不是声称已接管。观察到 PX4 离开 Offboard 后才锁存交出 Companion 输出。RC 切换请求与 Kill 严格区分。 | C05 GREEN；模拟 `nav_state=15` 状态完成接管；**RC 实际有效、厂商固件模式可用性 UNKNOWN** |
| C-H01 | `Core::dwell_land` 按严格递增的 `VehicleLandDetected` 样本 timestamp、真实观察时间跨度与最大样本间隔判断；冻结/倒退、长时间缺帧、反弹重置。覆盖 ArmedGround、liftoff、touchdown、Landed 和 ground-confirmed Disarm。 | C03 + 独立 frozen/touchdown/bounce + 现存完整飞行周期 GREEN；实际传感器频率 UNKNOWN |
| C-H02 | 恢复执行阶段 `PoweredGround / Descent / Landed` 与控制责任字段 `recovery_responsibility()` 明确区分；无法继续控制时显示 `PX4_FAILSAFE_DEPENDENT_NOT_VERIFIED_NO_COMPANION_OUTPUT`，**不声称 Companion 断电时能自动降落**。 | 软件动作分类已明确；PX4 real failsafe 必须另行配置只读核实和 SITL/无桨试验；**NO-GO 外部阻断未消除** |
| C-M01 | `Runtime::Pending` 增加 requested/published/sink_denied/wire_stamp/status_at_wire/land_at_wire。实际 ROS Node 构造 Runtime 后启用显式 sink commit；先完成最终 attitude sink，后才请求 176/400 command sink，失败记拒绝、不接受 ACK。保持基线独立 400/176/21 ACK 水印。 | sink-denied 竞态、先 ACK 后 commit、双 publisher 全链 fixture GREEN；真实 DDS/ROS UNKNOWN |
| C-M02 | `README_zh.md` 明确 r2 来自 B r1 (整包 SHA256)，r1 继承 0.4.3，0.4.1 是历史节点；`VERSION` 标记 r2。 | 文档修正完成 |

## 层级动作/控制权边界

- **计划中的动力撤除**：在 `PoweredGround` 连续发送受 final sink 检查的减推力 setpoint，正常优雅 shutdown 超过 60 秒时如果仍 armed+Offboard 且安全链健康，继续此流程而非主动退出；结束后依据递增 land detector 的 landed + at_rest + Z/VZ dwell 进入 `Landed`，待额外新样本证明后才请求 `Disarm`。`RequestDisarm` 仍需最终地面 sink 和有同事务 ACK / 新 armed 状态。
- **计划中的空中下降**：只在健康 Z/VZ/态度数据和当前 Offboard 可用时由 Companion 执行 `Descent`，任何无人机动力学稳定性都未经验证。弹跳时必须恢复发布，不在单次 `landed=true` 时断输出。
- **普通软件/遥控接管申请**：仅代表意图，不是 ACK。仍处于 armed+Offboard 时 Companion 保留受限控制责任；实测 PX4 mode 变化后 Companion 才可退出；本修复不自动发送未经审定的 armed 模式切换 176 指令。
- **Kill / 失去控制能力**：Kill 不会被普通接管语义覆盖；当传感器丢失/loop stall/程序崩溃时 Companion 不能保证继续发 setpoint，实际安全动作依赖 PX4 本机配置 **UNKNOWN**；`FaultClass` 仅用于诊断，不能凭日志标签宣称安全着陆。

## 本轮测试命令与证据语义

```bash
# 原独立反例（首次执行在 r1 上得到 C01–C05 RED；此文件被原样留存）
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I src/jikl_fc01_system/include -I tests \
  tests/c_independent_repro/C_M1_COUNTEREXAMPLES.cpp -o /tmp/c_m1_r2
/tmp/c_m1_r2

# 带断言的转译测试、时间戳复用、bounce、ACK/sink interleaving
bash scripts/test.sh
```

固定 r1 上五项 RED 日志和 r2 同一独立 repro 输出、assertions 原始输出和完整单次回归退出码见交付证据包。此环境 `colcon`、`/opt/ros/humble` 均不可用，所以 **原生 Humble colcon build/test = UNKNOWN**；类型 fixture `-fsyntax-only` PASS 不等于 ROS build/link PASS。

## 必须继续保持 NO-GO 的剩余风险

1. **ROS/PX4 真实链路未验证**：ROS2 Humble 原生构建、真实 `px4_msgs` pin、DDS 事件时序、FC01 厂商分支、负载、ARM64 构建均未测。
2. **飞控侧失联接管是外部阻断**：`COM_OF_LOSS_T`、`COM_OBL_RC_ACT`、RC/Offboard 模式、Kill，以及 `NAVIGATION_STATE_STAB` 是否可在没有 XY 定位时安全由 RC 操控，必须只读取证并经独立验证。PX4 文档指出 Offboard >2Hz proof-of-life 与失联后动作依赖参数设置；**代码不能在 Companion 崩溃时承诺降落**。
3. **垂直动力学未标定**：hover thrust、min/max、slew、起飞检测阈值、降落电机响应、电池电压下降、地面效应、接触弹跳都仅在模拟样本测试；无真实飞行许可。
4. **中断恢复能力**：在姿态、Z/VZ、模式或日志丢失时主动停止不可靠控制会失去控制责任；外部强制 SIGKILL、操作系统崩溃或断电时，优雅关闭的 60 秒延长逻辑也无法继续运行。必须通过 PX4 失效保护的 SITL/无桨台架实测，不得把 `Safety` 视为动作执行成功。
5. **模式切换真实性**：真实 PX4 mode 交接和 ACK/状态一致性仍需重新审查；软件接管请求不等于真实人工控制能力。
6. **Disarm 地面观测时效性**：最终 sink 使用最近有效的传感器缓存，已针对时间戳推进增加 dwell；真实 land detector 长延迟、异常边界和 ROS callback 同步仍需 C 独立审计。
7. **未获得独立 C r2 签字**。上述 PASS 只代表 B 使用合成输入执行的离线回归。

## 智能体 C r2 再审任务

1. 固定核对 r1 原包 SHA256、r2 全源 ZIP 哈希、r1->r2 patch 反向/正向重放和全部 SHA256 manifest，核实旧 `px4_msgs` 逐文件保持。
2. 独立运行 C 原始 5 组反例并复核 GREEN 语义；尤其检查首次 TIMEOUT tick 推力 slew、同一状态下冻结的 detector timestamp、ground confirm baseline、Land bounce 下心跳。
3. 从 `Runtime::request/step/ack/confirm_attitude_command_publication` 沿 `fc01_system_node.cpp::tick/publish_command` 复查最终 sink 顺序。重点检验 payload/ACK 先后、同一 command 400 的后发事务、水印追踪与 denial audit。
4. 检查真实导航状态确认而非 operator request 是否被误作 release；Kill / telemetry loss 不得被地面成功标签掩盖。
5. 在独立 **Ubuntu 22.04 / Humble** 开启原生 `colcon build/test`，随后才能提交下一阶段 SITL/无桨台架的另行审批。所有四个 flight gates 持续默认为 false。
