# C 独立安全审查正式任务书：R5 S10 DEV（不得代替飞行放行）

**审查件**：`0.4.4-offboard-flight-dev-r5-s10-dev`，独立离线 R5 候选；**基准**：用户 DATA 的 `S10_MODIFIED_SOURCE`，再与原始 R4 比对。保持真机完全断开。

## 审查前不可信项

B 的报告、测试与 log 都只能算 B 的自证；请 C 重新核对 SHA256、源码 diff、原始 C01–C12 文件字节、ROS topic/schema 与最终发布边界。任何 B 自测 PASS 不等于 C PASS，更不等于真实 PX4 SITL / FC01 PASS。

## 强制执行的独立检查

1. 在 `S10_TO_R5.patch` 中逐 hunk 追踪唯一修改的主控制 `attitude_altitude_core.hpp`。验证 `land_dwell_blocked_stamp_` 不能在不同 Phase 意外重置为可复用旧消息，`now`/stamp 缺失、倒退、冻结、1.25 秒上限、浮点边界以及 20 Hz 复用情况均 fail-closed。
2. 独立构造 C03 与本轮新的 1 Hz 正例（严格 1Hz sample stamp）、重复消息、旧 stamp 在缓存状态反转后反复调用、1.26 秒中断、新旧时间戳交错、`landed` 弹跳、`z/vz` 过阈值后回落、`at_rest=false`、状态过期与估计器重置。证明没有用同一帧 detector 来当两帧。
3. 将 `Core::step()` 从 `SelfCheck` 到 `Safety` 的所有 Phase 源码逐一列出，验证 `ThrustRamp → Ascend` 的必要且充分条件，`Ascend` 超时后 `PoweredGround/Descent` 的同周期推力不归零、不跃迁、不绕过 sink；新 `Ascend` 超时不能让 disabled gate、Kill、RC takeover、DDS stale、ACK ambiguous 重新获权。
4. **P0：独立重演真实 S10 类似场景**：`ThrustRamp → LIFTOFF_TIMEOUT_ACTIVE_POWER_RECOVERY → DESCENT_TIMEOUT_ARMED_OFFBOARD_PX4_HANDOFF_UNVERIFIED`，从 Output.publish，经 Runtime 到两个 Node sink，验证 20 Hz 心跳确实停止，报告真实 PX4 `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` 对 Position/Altitude/Land fallback 的逐事件证据。当前只允许离线仿真和静态检查；不可假定 SITL 已通过。
5. 审查 Timeout 信号类型分类是否误报 `Px4Handoff`、是否存在 armed=true/fresh/status 假设，`Safety` 后确认 `recovery_responsibility` 仍表达“PX4 负责但未验证”，不误报安全地面/Disarm 完成。
6. 确认未修改 R4/S10 原 RC：`manual_control_setpoint`、`manual_control_switches`，DDS、176/400/21 ACK 生命周期、Authority 选择/锁定、Command400 串行与隔离、四重门控、态度发布的两个 sink 与最后一刻边界、`px4_msgs` 文件。比对版本、脚本安装命名语义与 default-false 正式配置。
7. 真正 Ubuntu 22.04 ROS2 Humble native `colcon` build 和 ABI/DDS 检验，在无 ROS 环境则明写 **NOT RUN**；不得把 adapter fixture 当作 native build。x86_64 / ARM64 均须分别验证。
8. 必须保留独立 original C01–C12 反例进行失败-被修复对照，逐 case 报告 PASS/BLOCKER/UNKNOWN，不可改原始反例的断言或测试期望。验证本轮新增测试自身不是无效 oracle（强行把状态设真等）。
9. 对 A 的 `A_TO_B_FIX_CONTRACT.md` 建立专门增量 PR/review；本 R5 不含 A 所支持的动力参数修复。尤其核实 battery warning 和 PX4 Takeoff detected 事件在 ULog 中可否证明 M1 联合离地证据曾真实存在。

## 出具结果

- `C_R5_S10_INDEPENDENT_FINDINGS.md`：按 P0/P1/P2 给事实、源文件/行、复现命令、反例结果。
- `C_R5_S10_TEST_MATRIX.csv`：分列 static / pure core / ROS fixture / Humble native build / PX4 SITL / real FC01，未运行就是 NOT RUN。
- `C_R5_S10_RELEASE_VERDICT.md`：默认 NO-GO；尤其 P0 airborne handoff 未闭环、ULog 根因未确定、ROS build 未验证时禁止放行。

审查过程禁止连接真实 FC01、发送 Arm/Takeoff/Disarm 命令、写真实参数、覆盖 R4/S10 当前运行目录。
