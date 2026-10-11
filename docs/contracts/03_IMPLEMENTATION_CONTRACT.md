# 0.4 Implementation Contract

## 1. 基线不可破坏

必须从 `baseline_source/fc01_system_v0.3.1-ackfix_r2_source.*` 开始。

必须保留：

- 0.3.1 ACK correlation；
- ground MODE 测试及 `ground=PASS` 语义；
- stale/duplicate/in-progress ACK 防护；
- Unix socket 权限保护；
- 日志；
- Agent 脚本；
- install/update/rollback release 机制；
- ground profile；
- 消息版本 static_assert 思路。

不得把 0.3.1 直接覆盖；新版本必须为独立 release，例如：

`0.4.0-attitude-altitude-<timestamp>-<id>`

## 2. 新 profile

增加明确、互斥的控制 profile：

`attitude_altitude`

现有导航型 profile 与新 profile 不得同时拥有 PX4 setpoint ownership。

建议：

- ground：仅 ground transaction，不发布飞行 setpoint；
- flight/sim：保留原 Position/Velocity/TrajectorySetpoint 路径；
- attitude_altitude：只使用 `OffboardControlMode + VehicleAttitudeSetpoint`。

不要仅靠“虽然 publisher 存在但我不 publish”来表示互斥；尽量 profile-dependent 创建对应 setpoint publisher，并同步修改 publisher exclusivity 检查。

## 3. 新消息接口

### 发布

`/fmu/in/offboard_control_mode`

Attitude profile 必须：

```text
position=false
velocity=false
acceleration=false
attitude=true
body_rate=false
thrust_and_torque=false
direct_actuator=false
```

`/fmu/in/vehicle_attitude_setpoint`

使用 **px4_msgs v1.16 / MESSAGE_VERSION=0**。

第一阶段 setpoint：

- roll target = level
- pitch target = level
- yaw reference = 进入控制准备时从当前 `VehicleAttitude` 捕获的 yaw reference
- `thrust_body = [0,0,0]`（第一阶段绝不 Arm，且如果检测到 armed 必须退出/拒绝继续）

### 订阅

至少：

- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_land_detected`
- `/fmu/out/vehicle_command_ack`
- 继续保留当前安全所需 battery/failsafe topics

## 4. 第一阶段状态机（必须交付并默认运行这个阶段）

```text
IDLE
 -> SELF_CHECK
 -> PRESTREAM
 -> REQUEST_OFFBOARD
 -> OFFBOARD_READY
```

第一阶段：

- `flight_actions_enabled=false`
- `arm_enabled=false`
- `takeoff_enabled=false`
- **任何路径都不得自动发 Arm**
- **任何路径都不得发正的飞行 thrust**
- 如果外部/人工导致 `armed=true`，第一阶段节点必须拒绝继续控制，并产生显式错误状态；不要偷偷继续 zero-thrust Offboard。

PRESTREAM：

- 连续发送健康的 `OffboardControlMode(attitude=true)` + `VehicleAttitudeSetpoint`
- >= 1 s 后才允许 REQUEST_OFFBOARD
- 推荐 loop 20 Hz（现有主循环就是 50 ms），允许实现成 20–50 Hz

OFFBOARD_READY 验收：

- ACK transaction 成功；
- `VehicleStatus.nav_state == OFFBOARD`；
- `armed == false`；
- setpoint/heartbeat 连续；
- 无 publisher conflict；
- telemetry fresh；
- attitude finite/fresh；
- Z/VZ valid/fresh（即使第一阶段不用 thrust，也为第二阶段建立一致 gate）。

## 5. 第二阶段代码：可以实现，但必须默认锁死

实现但默认不可达：

```text
REQUEST_ARM
 -> ARMED_GROUND
 -> THRUST_RAMP
 -> ASCEND
 -> HEIGHT_HOLD
 -> DESCENT
 -> LANDED
 -> REQUEST_DISARM
 -> SAFE_DISARMED
```

禁止把 `VEHICLE_CMD_NAV_LAND` 当作唯一降落实现，因为当前没有 XY/VXY；建议第二阶段以 level attitude + vertical controller 做受控下降，`VehicleLandDetected` 确认 landed 后再发送 `VEHICLE_CMD_COMPONENT_ARM_DISARM(param1=0)`。

Arm 与 Disarm 都是 command 400；必须确保同一 command 的 transaction 不并发，freshness baseline 能区分顺序请求，并新增相应测试。

## 6. Vertical Controller（第二阶段）

内部统一“向上为正”：

```text
h  = z0 - z_ned
vh = -vz_ned
```

其中 `z0` 仅在 Disarmed + landed/at_rest + Z valid + 静置足够时间时建立/更新。

建议级联结构：

```text
height error -> vertical velocity target -> velocity error -> thrust correction
Tcmd = hover_thrust + correction
thrust_body = [0,0,-Tcmd]
```

必须有：

- finite checks
- integrator anti-windup（如果实现 I）
- thrust clamp
- thrust slew-rate limit
- max height
- max climb/descent rate
- telemetry timeout
- estimator reset handling

**不要猜最终飞行增益或 hover thrust。**

推荐用显式 sentinel / validation：只要 `flight_actions_enabled=true`，若 hover thrust、limits、gains 未被用户明确配置到合法范围，节点必须拒绝进入 Arm/Takeoff。

## 7. Yaw / attitude policy

当前 `Heading=NO`，所以不要做世界坐标绝对航向控制。

进入 attitude profile 时：

- 从 `VehicleAttitude.q` 捕获当前 yaw reference；
- setpoint roll/pitch 为 level；
- yaw target 保持该局部 reference；
- 允许 heading 漂移的工程现实，不把它当可靠 North heading。

`quat_reset_counter`：

- Disarmed/prepare 阶段变化：允许重新建立 reference，并记录事件；
- Armed/active 阶段变化：进入显式安全状态，停止宣称 healthy heartbeat/control；不得无声重基准继续飞。

## 8. Health-gated heartbeat

禁止做一个永远独立运行的假 heartbeat 线程。

只有以下条件全部成立才允许发送 Offboard heartbeat：

- supervisor healthy
- attitude fresh + finite
- local Z/VZ fresh + valid（attitude_altitude profile）
- no time regression/reset hazard
- no PX4 failsafe
- publisher ownership/exclusivity 正常
- outputs finite

否则明确失去 control readiness，让 PX4 的 Offboard-loss 机制接管。

F 不得在本任务里擅自写死 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` 数值；这些参数要在真机下一阶段单独确认。

## 9. Operator API

不要把 `takeoff` 命令直接改成“只要输入就飞”。建议新增分层命令，例如：

- `status`
- `mode`（现有 ground test，必须继续工作）
- `attitude-ready` / `prepare-offboard`（第一阶段允许）
- `takeoff <height>`（第二阶段，受 master gate 拦截）
- `land`
- `stop`

单次 `--command` 仍不得允许危险飞行动作；危险动作只允许 live interactive console + 显式配置 gate 双重成立。

## 10. 消息版本

新增 static_assert：

- `VehicleAttitude::MESSAGE_VERSION == 0`
- `VehicleAttitudeSetpoint::MESSAGE_VERSION == 0`

保留已有 Status/Local/Land/Ack/Command/TrajectorySetpoint 的 v1.16 约束。

## 11. 不属于 F 的任务

不要研究/修改：

- PID tuning
- 2/4 电机历史不对称
- DShot telemetry
- ESC firmware
- SENS_BOARD_* 参数
- Optical Flow/VIO 实现
- Orange Pi 系统安装

只需保证代码结构将来 ARM64 可构建。
