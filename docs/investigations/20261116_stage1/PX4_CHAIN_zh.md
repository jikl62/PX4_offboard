# Stage 1：PX4 v1.16 高度链路与 Offboard 丢失判据（只读）

核验时间：2026-11-16。源码目录：`/home/jikl62/fc01_px4_dds_build/PX4-DM-FC01-v1.16`。

## 固件身份及证据边界

- Git HEAD `07983bfb894b9372d7520d43ac10dc792d0b4b9f`，与已解析 Nov15 ULog 中 `ver_sw` 一致。
- 工作区两处修改：`src/modules/uxrce_dds_client/dds_topics.yaml` 增加 `/fmu/out/manual_control_switches`；`uxrce_dds_client.cpp:542` 将一次 ping 的同步等待从 1000 ms 改成 0。除此之外 status 没有列出修改。
- EKF2 与 Commander 下述源文件未修改，能作为匹配提交的语义依据。HEAD 相同并不能证明这两处未提交 DDS 修改已经刷入当前飞控；需要实际固件构建身份或同次连接证据。
- 不使用另一个 `/home/jikl62/PX4-Autopilot` 的 v1.18 开发分支解释本次 v1.16。

## EKF2 → VehicleLocalPosition

`src/modules/ekf2/EKF2.cpp`：

| 位置 | 实际含义 |
| --- | --- |
| 786–789 | 仅在 `_ekf.update()` 成功后调用 `PublishLocalPosition(now)`。 |
| 1538–1548 | `timestamp_sample=timestamp`；`z` 来自 EKF fused local NED position(2)，单位 m，向下为正。此处 sample stamp 是传入本轮 now，不能直接视作测距仪原始采样时间。 |
| 1550–1557 | `vz` 为 EKF NED velocity(2)，单位 m/s，向下为正；`z_deriv` 是 EKF vertical position derivative。两者不是同一接口字段，不能强行等同。 |
| 1570–1571 | 为兼容下游，`z_valid` 与 `v_z_valid` 均由垂直位置有效 **或** 垂直速度有效置 true；这不是两个彼此独立的原始传感器质量证明。 |
| 1603–1606 | `dist_bottom` 属 terrain/HAGL estimate，另有 `dist_bottom_valid`；不能只因数值合理而当作可靠离地高度。 |
| 1624–1625 | 同时发布 `delta_z/z_reset_counter` 和 `delta_vz/vz_reset_counter`。 |
| 1657–1658 | `timestamp` 为本次发布的 PX4 hrt 时间（replay 模式例外）。 |

`msg/versioned/VehicleLocalPosition.msg:1–35` 规定 NED/单位/计数器；`59–65` 另列底部地形距离，不等于 `z`。

## DDS 时间与收发

- `src/modules/uxrce_dds_client/dds_topics.yaml:68–69` 配置 fused local position 发到 `/fmu/out/vehicle_local_position`。
- `dds_topics.h.em:122–138` 把 `session->time_offset/1000`（ns→µs）传给序列化函数。
- `Tools/msg/templates/ucdr/msg.h.em:138–144` 对 `timestamp`、`timestamp_sample` **都加** time offset，输出到 Agent 的时间域。
- 反向 `/fmu/in/offboard_control_mode` 在 `dds_topics.yaml:115–116`；`dds_topics.h.em:170–184` 反序列化后发布 uORB。
- 反序列化模板 `Tools/msg/templates/ucdr/msg.h.em:167–169`：零戳用当前 hrt；非零戳减 offset 并限制不晚于当前 hrt。
- 所以 ULog 的 PX4 boot µs、ROS 收到的同步后 µs、香橙派 monotonic 秒、墙钟日期不能直接相减。需同一次 boot/同一连接的源戳锚点或 timesync 对齐。

## Offboard 停止与 “No offboard signal”

`src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:38–65`：

1. 每次检查先把 `offboard_control_signal_lost=true`。
2. 如有 `offboard_control_mode`，检查 `hrt_absolute_time() < message.timestamp + COM_OF_LOSS_T * 1s`（46–47）。
3. position / velocity / acceleration / attitude / body_rate / thrust_and_torque / direct_actuator 至少一个使能，且消息仍新鲜，才有基本可用性（49–51）。
4. position 模式另要求 local_position_valid；velocity 模式要求 local_velocity_valid；acceleration 模式要求 attitude_valid（53–61）。
5. `offboard_control_signal_lost = !offboard_available`（65）。本 FC01 attitude 心跳不会走 position/velocity 这两项条件，但还需整机模式自身要求的姿态健康。

**这个确切版本不能套用“固定 500 ms 心跳超时 + 再等 COM_OF_LOSS_T”的说法。** 此文件直接使用 COM_OF_LOSS_T 判定消息年龄。`commander_params.c:311–322` 默认 1.0 s；实际飞行应以同次 ULog 生效值为准。`Commander.cpp:1839–1844` 通常按 10 Hz 运行健康检查，状态变化可提前；诊断调度及消息日志采样会影响观察到的边缘时刻。

`modeCheck.cpp:133–141`：当 offboard lost 且存在需 Offboard 的模式位时，报告事件 `check_modes_offboard_signal`，文字为 `No offboard signal`，并清除相关 can-run 位。它不是串口坏了的特异诊断，也不是无条件证明机体断电。

`failsafe/failsafe.cpp:645–656`：用户期望模式需要 Offboard 且 lost，选择 `COM_OBL_RC_ACT`；`fromOffboardLossActParam:280–324` 映射 Position / Altitude / Stabilized / RTL / Land / Hold / Terminate / Disarm，再受其余健康条件约束。不要未经同次参数核验就宣称落地动作。

`Commander.cpp:2952–2958` 收到新 Offboard 控制模式时可触发尽快重检查，以允许模式激活。`HealthAndArmingChecks.cpp:102–107` 的 500 ms 是 **failsafe_flags 定期发布的间隔条件之一**，状态改变会直接发布；它不是额外的 Offboard 丢失延时。

### 可验证因果与未证实现场链

源码允许的链为：FC01 锁存安全事件→当轮/后续停止本地目标及 attitude 心跳发布→若 PX4 未收到其他有效发布者补充的心跳，其最后有效 OffboardControlMode 戳超出 COM_OF_LOSS_T→lost flag/事件/模式回退。这个链条不等于已经证实当前异常试飞就是如此。

要证明现场先后顺序，至少需要同次：控制器原始阶段/原因/源戳，最后一次最终 sink 提交，以及 ULog 中 OffboardControlMode/vehicle_status/failsafe_flags/event/attitude reset 记录。进程内 sink 返回成功不等于 PX4 实际接收；ULog 最后记录样本也受记录采样策略影响。

Nov15 已知 ULog COM_OF_LOSS_T=1.0、COM_OBL_RC_ACT=1，但该短片段为 Stabilized，无 OffboardControlMode topic，不能用于测算本次 FC01 停发到 No offboard signal 的延迟，更不能代替当前 Position 实飞基线。

## 源文件 SHA256（当前可核对副本）

```text
c032e72d2ff4582a319baa7cc44016ea5e7869fc8bd0ad5630a46fe6e5cf6bd5  src/modules/ekf2/EKF2.cpp
36890c99b6fb9185fa153c6738112108d9bc8398ac6a9138de7e7ad78db53c7d  src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp
988f1fd5f63479d99f7218fe31cec55d8488c26cd285b1535c7d0b02b2f4affe  src/modules/commander/HealthAndArmingChecks/checks/modeCheck.cpp
db3a53c346fb7d5d358f76edf55bb7a4ccd1343b8e2302c210be2fa79e01aed6  src/modules/uxrce_dds_client/dds_topics.yaml
3c762dbed4e4153722e4d854bf175c0b2eed6e1a335f6d187b22c5e456aa41a8  src/modules/uxrce_dds_client/uxrce_dds_client.cpp
2a434467a412c3dbe441ea5b7fe9d8918c4ba70721b920ee39e2ebe246bbdd8e  Tools/msg/templates/ucdr/msg.h.em
```

## 只读复核命令

```bash
cd /home/jikl62/fc01_px4_dds_build/PX4-DM-FC01-v1.16
git rev-parse HEAD
git status --short
git diff -- src/modules/uxrce_dds_client/dds_topics.yaml src/modules/uxrce_dds_client/uxrce_dds_client.cpp
rg -n 'No offboard signal|offboard_control_signal_lost|COM_OF_LOSS_T' src/modules/commander
nl -ba src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp | sed -n '38,67p'
nl -ba src/modules/ekf2/EKF2.cpp | sed -n '1538,1658p'
nl -ba Tools/msg/templates/ucdr/msg.h.em | sed -n '127,175p'
```

未修改生产源码、飞行配置或 PX4 参数，未访问硬件、未运行控制服务、未部署。
