# 已验证基线（2026-10-04）

## 已知良好 release

- 当前已验证版本：`0.3.1-ackfix-20261004T090116Z-Pdwbao`
- `VERSION`: `0.3.1-ackfix`
- Ubuntu x86_64 + ROS2 Humble 完整 `colcon build`：PASS
- ACK lifecycle 单元测试：PASS
- `ground.yaml`: `flight_enabled: false`
- `flight.yaml`: `flight_enabled: false`
- `sim.yaml`: `flight_enabled: true`

## px4_msgs 固定版本

实际使用 commit：

`392e831c1f659429ca83902e66820d7094591410`

已在真实环境打印确认：

### VehicleAttitudeSetpoint

- `MESSAGE_VERSION = 0`
- `timestamp`
- `yaw_sp_move_rate`
- `q_d[4]`
- `thrust_body[3]`
- `reset_integral`
- `fw_control_yaw_wheel`

### OffboardControlMode

- `position`
- `velocity`
- `acceleration`
- `attitude`
- `body_rate`
- `thrust_and_torque`
- `direct_actuator`

### VehicleAttitude

- `MESSAGE_VERSION = 0`
- `timestamp`
- `timestamp_sample`
- `q[4]` — FRD body -> NED earth，Hamilton `(w,x,y,z)`
- `delta_q_reset[4]`
- `quat_reset_counter`

## DDS / ROS2 通信

真机已验证：

- FC01 `uxrce_dds_client` ↔ Micro XRCE-DDS Agent：session established
- FC01 → PC telemetry：稳定接收
- PC → FC01 `VehicleCommand`：真实执行
- QGC/VehicleStatus：Hold(mode=4) → Stabilized(mode=15)
- `VehicleCommandAck`：真实返回

## 已验证 ACK transaction

真机成功 ACK：

```json
{"command":176,"result":0,"target_system":1,"target_component":191,"from_external":false}
```

并且共享 ACK 流中真实存在其他 ACK，例如：

```text
command=512 result=2 target=255/190
command=401 result=0 target=255/190
```

因此 0.3.1 已验证的 ACK matching 规则必须继承：

- 必须有 pending request；
- `ACK.command == pending.command`；
- `ACK.target_system == request.source_system`；
- `ACK.target_component == request.source_component`；
- freshness / timestamp baseline 只作旧 ACK 防护，不作 transaction ID；
- 同一 `(command, source_system, source_component)` 不允许不可区分的并发 outstanding request；
- `result=0` ACCEPTED；
- `result=5` IN_PROGRESS，不能终止；
- 其他终态为失败；
- timeout 保留；
- **不得要求 `ACK.from_external == true`**。

真机 ground transaction 最终已达到：

```text
mode=15
armed=False
ground=PASS
reason=ACK_ACCEPTED_AND_NEW_STABILIZED_STATUS
```

## 当前估计器事实

真机当前：

```text
XY       = NO
VXY      = NO
Heading  = NO
Z        = YES
VZ       = YES
```

因此：

- 旧 Position Offboard 当前不可作为首个真机控制方案；
- 旧 Velocity Offboard 当前不可作为首个真机控制方案；
- 不得通过删除 validity gate 强行复用旧导航控制器。

## 姿态接口验证

ROS2 `VehicleAttitude` 六方向人工测试已经确认：

- 抬机头 → pitch 正；
- 压机头 → pitch 负；
- 右侧压低 → roll 正；
- 左侧压低 → roll 负；
- 右转 → yaw 增加；
- 左转 → yaw 减少；
- `quat_reset_counter` 测试期间保持 0。

桌面/机架并非严格水平，因此静态几度 pitch offset **不作为 0.4 编码阻塞项**，F 不得擅自修改 `SENS_BOARD_*` 参数。

## 当前硬件/参数背景（仅上下文，不要求 F 调参）

- DAMIAO DM-FC01 / PX4 1.16.0 厂商固件
- QGC 当前 motor geometry：四旋翼
- MAIN1..4 → Motor1..4
- DShot300
- `SENS_BOARD_ROT = Yaw 180°`
- `SENS_BOARD_X_OFF = -1.037°`
- `SENS_BOARD_Y_OFF = +4.306°`
- `SENS_BOARD_Z_OFF = 0°`
- DShot telemetry 当前未启用；与本任务无关

## 当前项目目标

当前主目标不是飞控 PID/ESC 调参，而是：

> Ubuntu PC 上的程序通过 ROS2 + Micro XRCE-DDS + FC01/PX4，完成 Attitude Offboard 控制链；在审查放行后实现自主 Arm、垂直起飞、定高、下降、Disarm。相同源码之后迁移 Orange Pi Zero 3W A733。
