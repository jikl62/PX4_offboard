# M1 候选件交智能体 C 独立安全审查

**交付性质：智能体 B 的实现与自审材料，不是智能体 C 已完成的独立审查结论。**

候选版本：`0.4.4-offboard-flight-dev`。
执行时间：2026-10-09。标记：**DEV CANDIDATE / NOT CLEARED FOR FLIGHT**。

请执行独立源码差异审查（不接受 B 的测试通过作为唯一依据）：

1. 统一 diff 与基线逐文件比较，`px4_msgs` 不得存在任何差异；核查 RC source `manual_control_setpoint`、switch/Kill `manual_control_switches`、旧 176/400 ACK matcher、authority 锁、ROS 话题、双 sink 发布时序均未放松。
2. 逐个穿透状态机 `SelfCheck/Prestream/RequestOffboard/OffboardReady/RequestArm/ArmedGround/ThrustRamp/Ascend/HeightHold/Descent/Landed/RequestDisarm/SafeDisarmed/ReleaseOffboard/Idle`；尤其检查 `armed && at_rest=false`、离地计时误报、触地反弹、地面已确认、空中未知/自检超时退场。
3. 比较 Node 在 `runtime.step` 与 `publish_command / publish_attitude_at_boundary` 的双重审查间的状态采样；确认 400 Disarm 的 sink policy，176 release 的 ground-only 限制，Release 阶段零心跳与状态切换后停心跳，不允许旧 ACK 串扰。
4. 以**相同编译环境**对候选件运行真正 ROS2 Humble `colcon`/link 和 DDS 消息 compile/ABI 检查。当前的 Python adapter fixture 不能算这一步。
5. 独立测试 90° roll、yaw 注入、stage-one 非零、stale status/land、RC Kill、模式退出、Z/VZ discontinuity、ACK late/stale、STOP queued before 176 status、Landing bounce、arm/disarm 400 串行和 PX4 handoff。
6. 对真机后续工作只写验收计划，不可视此交接为允许连接 FC01、改参数、Arm、Takeoff 或刷固件的许可。

### B 的当前判断

- **CONFIRMED**：14 个 C++ 离线回归二进制、18 项 Python 测试、fixture adapter syntax PASS；至少一项原件 FAIL-before/DEV PASS-after 漏洞由外置独立 probe 复现。
- **INFERRED**：新状态机可以在合成时序下维持正确的 arm/disarm/176 权限和接地 handback 路径；与真实 PX4 观测仍可能存在差异。
- **UNKNOWN**：候选 x86_64/ARM64 ROS 2 原生构建；厂商固件模式、ACK、飞行失联保护；气动推力、动力余量、接地估计。

建议 C 对每一项关键行为出具独立 **PASS / BLOCKER / UNKNOWN**，严禁把 UNKNOWN 写成 PASS。尤其注意原始基线构建日志不是本候选构建记录。请以 `M1_AGENT_B_DEV_CANDIDATE_RELEASE_NOTES_20261009.md` 为风险矩阵起点，但不直接采信。
