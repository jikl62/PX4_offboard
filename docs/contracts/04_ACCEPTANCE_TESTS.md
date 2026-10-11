# F 交付验收测试

## A. 必须新增的纯 C++ 单元测试

至少覆盖：

### Attitude / quaternion
- identity / level quaternion；
- yaw reference 捕获；
- roll/pitch level + fixed yaw 输出；
- quaternion finite/normalized；
- `quat_reset_counter` disarmed rebase；
- active reset -> safety state。

### Vertical controller
- NED z/vz 到 up-positive h/vh 符号；
- height error -> correct thrust correction direction；
- hover feed-forward 存在；
- clamp；
- slew rate；
- anti-windup（若有 I）；
- stale/invalid Z/VZ -> no flight output；
- reset counter change handling；
- NaN/Inf refusal。

### State machine
- IDLE -> SELF_CHECK -> PRESTREAM；
- prestream <1s 不得请求 Offboard；
- healthy >=1s 才允许 REQUEST_OFFBOARD；
- ACK + new VehicleStatus 才到 OFFBOARD_READY；
- ACK accepted 但 status 未变不得 PASS；
- status 已变但 ACK 不合法不得 PASS；
- `flight_actions_enabled=false` 时所有路径都不得产生 Arm；
- `arm_enabled=false` / `takeoff_enabled=false` 时拒绝；
- 人工 armed 在 stage-1 -> explicit refusal/safety；
- same command=400 sequential arm/disarm correlation；
- duplicate/stale/wrong-target/wrong-command ACK；
- IN_PROGRESS -> ACCEPTED；
- timeout。

### Ownership
- attitude profile 不拥有/发布 TrajectorySetpoint；
- nav profile 不拥有/发布 VehicleAttitudeSetpoint；
- conflict publisher -> refuse ownership。

## B. Regression

0.3.1 tests 必须继续 PASS：

- `ack lifecycle tests: PASS`
- ground `MODE` transaction 逻辑未破坏
- socket security 未破坏
- config/release/update/rollback 未破坏

## C. Build

用户环境：Ubuntu x86_64 + ROS2 Humble。

必须给出可执行：

```bash
bash scripts/test.sh
bash scripts/install.sh <px4_msgs-v1.16-source>
```

不得自动切 `current`。

## D. 第一阶段真机 smoke test（F 只写步骤，不要求用户现在执行）

必须保证步骤只到：

```text
Agent session
controller attitude_altitude profile
status
prepare-offboard / attitude-ready
PRESTREAM
Offboard ACK
nav_state OFFBOARD
armed=false
OFFBOARD_READY
```

必须明确要求：

- props removed；
- `flight_actions_enabled=false`；
- `arm_enabled=false`；
- `takeoff_enabled=false`；
- 验收结束不得继续 Arm。

## E. 第二阶段禁止默认执行

即便已实现 Arm/Takeoff 状态机，交付文档必须把它列为“主控审查后才启用”。不能把安装后下一条命令写成 takeoff。
