# 正式任务书——智能体 F：FC01 0.4 Attitude-Altitude 实现工程师

## 角色

你负责在**已经真机验证的 `0.3.1-ackfix` 基线**上实现下一版：

`0.4.0-attitude-altitude`

这不是理论设计任务。你必须阅读实际源码、A/B/C 报告和手册，然后产出完整可审查源码包。

## 总目标

最终链路：

```text
Ubuntu PC / future Orange Pi
  -> ROS2 Humble
  -> Micro XRCE-DDS Agent
  -> DM-FC01 / PX4 1.16
  -> Attitude Offboard
  -> (审查放行后) autonomous Arm
  -> vertical takeoff
  -> height hold
  -> controlled descent
  -> disarm
```

当前没有有效 XY/VXY/Heading，但 Z/VZ 有效。因此第一版不使用 Position/Velocity Offboard 起飞，而增加 `attitude_altitude` profile：

- companion computer 做 Z/VZ 高度外环；
- PX4 继续做 attitude loop、rate loop、control allocation；
- 不直接控制电机。

## 绝对要求

1. 从 `baseline_source/fc01_system_v0.3.1-ackfix_r2_source.*` 出发；
2. 不破坏 0.3.1 已真机验证的 ACK transaction；
3. 固定 `px4_msgs` v1.16 接口；
4. 新增 `VehicleAttitude` + `VehicleAttitudeSetpoint`；
5. profile setpoint ownership 必须互斥；
6. 第一阶段默认绝对禁止自动 Arm/Takeoff；
7. 第二阶段可以实现完整 Arm/Takeoff/Hold/Descend/Disarm，但默认锁死；
8. 所有危险能力必须由配置 gate + 状态机 gate 双重限制；
9. 同一份源码以后可以在 Orange Pi ARM64 重新 build；
10. 不要研究飞控 PID、ESC、2/4 电机、历史 ULog。

## 第一阶段必须能做到

在 props removed、disarmed 条件下：

```text
IDLE
 -> SELF_CHECK
 -> PRESTREAM (>1 s)
 -> REQUEST_OFFBOARD
 -> OFFBOARD_READY
```

并证明：

- `OffboardControlMode.attitude=true`；
- `VehicleAttitudeSetpoint` 连续发送；
- 当前 yaw 被捕获为局部 reference；
- roll/pitch setpoint 为 level；
- 第一阶段 thrust 为 0；
- PX4 ACK 被现有 transaction 正确认领；
- VehicleStatus 真实进入 OFFBOARD；
- `armed=false`；
- 无 TrajectorySetpoint / AttitudeSetpoint ownership 冲突；
- `flight_actions_enabled=false` 时任何路径都不可能发 Arm。

## 第二阶段实现（默认 disabled）

```text
REQUEST_ARM
 -> ARMED_GROUND
 -> THRUST_RAMP
 -> ASCEND
 -> HEIGHT_HOLD
 -> DESCENT
 -> LANDED
 -> REQUEST_DISARM
```

高度：

```text
h  = z0 - z_ned
vh = -vz_ned
```

thrust：

```text
Tcmd = hover_thrust + vertical_feedback
thrust_body = [0,0,-Tcmd]
```

但不要猜实际 gains/hover_thrust。只有用户明确给出并通过 validation 后，master flight gate 才可打开。

## Safety

必须显式处理：

- stale attitude
- stale/invalid Z/VZ
- NaN/Inf
- quaternion reset
- time regression
- PX4 failsafe
- lost DDS/telemetry
- publisher conflict
- output saturation
- control loop stall
- unexpected armed state

Heartbeat 必须 health-gated；控制器坏了时不得由独立线程继续伪造“健康 Offboard”。

## Operator API

保留 `status` / `mode` ground test。

新增一个**不 Arm**的准备命令，如：

`attitude-ready`

或同等清晰命名。

危险 `takeoff` 只能在 live interactive console 且所有配置 gates 开启时接受；单次 `--command takeoff` 仍应拒绝。

## 版本与发布

不得覆盖旧 release。

新包至少包含：

```text
VERSION
README_zh.md
src/
config/
scripts/
tools/
tests/
```

更新 `package.xml` version；portable export 名称不能残留 0.3.0/0.3.1。

生成：

- source `.tar.gz`
- source `.zip`
- unified diff against 0.3.1 baseline

## 测试

严格执行 `04_ACCEPTANCE_TESTS.md`。

现有 ACK 单测必须回归 PASS，并新增 attitude-altitude 核心测试。

## 输出给主控

最终回答必须按以下结构：

1. 读到的基线与关键事实；
2. 架构修改摘要；
3. 文件级 diff 摘要；
4. 安全门设计；
5. 测试结果；
6. 未验证事项（必须明确 UNKNOWN）；
7. Ubuntu build/install 命令；
8. **第一阶段 props-off、绝不 Arm 的真机 smoke test**；
9. 第二阶段启用前需要主控审查的参数/条件；
10. 下载完整 source tar.gz/zip/diff。

如果输入材料之间冲突，先列出冲突并按 `00_README_FIRST.md` 的权威性排序处理，不要默默自行选一个。
