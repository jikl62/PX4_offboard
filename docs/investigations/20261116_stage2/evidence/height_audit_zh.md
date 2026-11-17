# 阶段 2 高度链独立只读审查

审查基线：`3677b9dd1f35e092e43501d26164c6263545c947`，分支 `readability/fc01-chinese-20261111`。本辅助代理没有改动生产源码、配置或用户未跟踪文件，没有硬件连接。

## 结论

已复现一项离地关联期限缺陷：在没有新 local 源戳时，已超过 land_timeout 的旧上升证据仍可与新 detector 拼成离地。最小候选与 Runtime 复现见后附修订。除此之外，融合 z/vz、本次安全地面 z0 和 z0-z 路线符合用户要求，不宜重写。测距趋势诊断应保持观测性质。

## 核对位置

- `fc01_system_node.cpp:1442 populate_attitude_feedback` 将 `vehicle_local_position` 的融合 `z/vz/z_deriv` 和 reset/有效位/两种源戳原样映射；NED 向下正。envelope、原始 sample 及主机进展寿命分别验证。`source_clock.hpp:86 receive_sampled_source` 不给旧 raw sample 续鲜。
- `rc_navigation.hpp:434 observe_ground_reference` 要求未解锁、非 Offboard、人工解锁开关关闭、Stabilized、landed/at_rest、新鲜有效 z/vz、静止足够、有限值、低 vz、源戳推进及同一估计参考；至少 3 local、3 land、2 status 和跨 2 s 窗口。慢漂移可在该窗口更新本次锚点，时间回退/reset/同戳改数破坏资格。
- `rc_navigation.hpp:534 capture` 保存当前安全地面锚点。只由 Prestream/WaitArm 无解锁意图分支调用；arm intent 后冻结，不在已解锁状态重新归零。
- `attitude_altitude_core.hpp:2717 rc_startup` 继承上述锚点；Joined 时 `target_=c_.rc_target_height`，`command_height_=0`。现场实际 `rc_target_height=1.5 m`，但配置仍是目标唯一来源。
- `attitude_altitude_core.hpp:1442` 与 `vertical_controller.hpp:step` 使用同一 `height=z0-z`、`up_speed=-vz`，上升和定高都不把 absolute z 当相对高度。
- `attitude_altitude_core.hpp:2092 advance_thrust_ramp` 高度达到 min(target,liftoff_height) 可提前进入同一闭环并受物理 ramp ceiling；联合离地确认后转 Ascend，不重置轨迹、积分或 slew。时间/推力达到计划值不等于离地。
- `takeoff_evidence.hpp:observe` 分别累计新 detector 和新 local 的源/主机时间及实际高度增长；重复缓存不能伪造离地，允许有界 detector 迟到桥接。
- `attitude_altitude_core.hpp:2490 height_feedback` 统一更新轨迹/速度前馈并调用原 PI；没有新推力参数。
- `attitude_altitude_core.hpp:1689` 到高要轨迹到任务目标、离地已证、非 landed、实测相对高度/向上速度均在容差且连续 settle_seconds（现场 2 s）。未稳定经过 1.5m 或超出 0.15m 容差应不进入 HeightHold；Ascend 10s 超时走原受监督下降。
- `vertical_feedback_guard.hpp` 的 abs(vz-z_deriv) 单独持续偏差仅 suspect/rate_warning；位置与导数积分不符、非有限输入、源时间不连续仍是硬保护。

## 缺口与建议

原节点未接 `dist_bottom` 或 `/fmu/out/distance_sensor`，因此当前日志不能核对测距上升趋势。`VehicleLocalPosition.dist_bottom` 是 PX4 底面/地形距离估计，带 valid、variance、reset、sensor bitfield；range bit 只说明参与过 range，不认证 MTF-02P，也非完全独立于 EKF 的原始测量。若父代理增加此诊断，应要求新鲜有效、有限非负、range bit、同一 reset 与新样本，缺失时明确 unavailable，不能替代 z/vz、修改 z0、认证离地、授予控制权或成为视觉关闭任务的新硬依赖。当前没有现场数据支持设新的硬一致性阈值。

到高 dwell 目前读取新鲜的连续缓存而非每帧源戳独立计数；最大原始寿命 0.5s 小于 2s 稳定窗口，完全冻结样本无法单独满足整个窗口。没有重放证据证明现状导致实际错误，不建议在本轮无依据改变。

## 本代理实际复跑

输出目录 `/tmp/fc01-stage2-height-audit-tests/`，命令和输出见 `results.json`。以下 5 个 C++ 测试全部 g++ C++17 O2 Wall Wextra 编译退出 0、运行退出 0：

1. `test_r632_height_tracking`：`R632_HEIGHT_TRACKING_PASS all`。
2. `test_rc_takeoff_stage`：`RC_TAKEOFF_1P5_STAGE_PASS`。
3. `test_takeoff_evidence`：`TAKEOFF_SOURCE_JOIN_PASS all`。
4. `test_rc_navigation`：ground offsets/drift/recovery、reset/timestamp、诊断 velocity、production no-vision 1.5m 生命周期、完整契约全部 PASS。
5. `test_vertical_feedback_guard`：`VERTICAL_FEEDBACK_GUARD_PASS (7 groups)`。

以上是合成反馈的控制／权限回归，不是动力学证明、全节点 ROS 回放或 ARM64 实测。本辅助代理未运行 ASan/UBSan。

## 阶段 3 可复核离线命令

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
g++ -std=c++17 -O2 -Wall -Wextra -I native/src/jikl_fc01_system/include -I native/tests native/tests/test_rc_navigation.cpp -o /tmp/fc01-stage3-rc
/tmp/fc01-stage3-rc
g++ -std=c++17 -O2 -Wall -Wextra -I native/src/jikl_fc01_system/include -I native/tests native/tests/test_r632_height_tracking.cpp -o /tmp/fc01-stage3-height
/tmp/fc01-stage3-height
```

现场仍缺：当前稳定 Position 同配置真实 ULog、最新版异常任务同 boot/PID 双端完整日志、range 有效／质量与是否融合证据、reset delta 与 PX4 最后实际接收心跳序列。阶段 1 的旧日志或稀疏导出不能证明当前保护误报。

## 后续定向审查修订：确认 P2 离地关联窗失效缺陷

父代理任务文件 `/tmp/fc01-stage2-height-task.txt` 明确要求进一步检查无新 local stamp 时的 rise 期限。已在 helper 和真实 Runtime 两个层级复现。

根因：`takeoff_evidence.hpp:85` 仅在 `local_stamp > local_seen_` 分支中检查 `now-motion_at_ > land_timeout`。如果最后真实上升已满 2.5s，但最新静止 local 缓存仍在 0.5s 寿命内，此时第二条 detector 到达且未带新 local，最终交集可以错误保留 rise_confirmed。违反文件自身声明的“只在有限 land_timeout 内桥接运动”合同。

可重复序列（helper）：t=0 和 .25s 连续上升形成 rise；持续新静止 local 到 t=2.5；t=2.5 首条 !landed detector；t=2.8 第二条 detector，local 仍是 t=2.5。该缓存年龄仅 .3s，但真实 motion_age=2.55s。基线得到 `rise=1 detector=1 confirmed=1`，复现程序按期望“不确认”返回 1。

同样输入通过现有 `GuardFlight` 的真实 Runtime，在 ThrustRamp 后 8 条上升 local、45 条静止新 local、再 .3s 冻结 local 并送两条 detector，可得到 `after_expiry rise=1 detector=1 confirmed=1 phase=9 reason=ASCEND`。因此并非仅 helper 的不可达输入。

复现源码：

- `/tmp/fc01-stage2-rise-expiry-repro.cpp`（helper）。
- `/tmp/fc01-stage2-rise-expiry-runtime.cpp`（真实 Runtime）。
- `/tmp/fc01-stage2-rise-expiry-boundaries.cpp`（2.5s 期限恰好通过、+1us 拒绝、local 冻结超期拒绝）。

隔离候选 `/tmp/fc01-stage2-rise-expiry-candidate.patch` 在最终交集前每次检查主机 motion 期限；源域期限仍只用新 local 检查，阈值不改。候选 include 放在 `/tmp/fc01-stage2-rise-candidate/`，**尚未应用到生产工作区**。Runtime 复现改为 `rise=0 detector=1 confirmed=0 phase=8 reason=RAMP_HEIGHT_GUARD_WAIT_NEW_LIFTOFF_EVIDENCE`，返回 0。精确边界和 +1us 测试均返回 0。

候选覆盖 include 下独立运行 5 个既有回归：takeoff_evidence、r632_height_tracking、rc_navigation、rc_takeoff_stage、r632_ramp_height_guard，全部编译／运行 0；命令和完整输出 `/tmp/fc01-stage2-rise-candidate-tests/results.json`。有效关联窗内 detector 迟到、1.5m 生命周期、原 RC/推力/高度轨迹均通过。

严重度 P2：短窗口内可能错误把已失效的旧运动关联到检测器，从 Ramp 提前转 Ascend；它不会新增人工解锁或 Offboard 权限，也不能单独绕过最终 HeightHold 稳定门。属于已确认代码缺陷，不能据此认定是现场旧任务故障根因。

复现命令：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -I native/src/jikl_fc01_system/include -I native/tests /tmp/fc01-stage2-rise-expiry-runtime.cpp -o /tmp/fc01-stage2-rise-expiry-runtime
/tmp/fc01-stage2-rise-expiry-runtime
# 基线预期退出1，输出ASCEND。
g++ -std=c++17 -O2 -Wall -Wextra -I /tmp/fc01-stage2-rise-candidate -I native/src/jikl_fc01_system/include -I native/tests /tmp/fc01-stage2-rise-expiry-runtime.cpp -o /tmp/fc01-stage2-rise-expiry-runtime-fixed
/tmp/fc01-stage2-rise-expiry-runtime-fixed
# 候选预期退出0，保持Ramp等待新离地证据。
```
