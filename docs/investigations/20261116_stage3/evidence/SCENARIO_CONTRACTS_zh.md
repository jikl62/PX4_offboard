# Stage 3 故障注入合同独立只读复核

源码 HEAD：`8767608f79a56e346d5654228770f948841fe0e3`。本子任务仅源码阅读，没有启动 ROS、PX4、串口、控制进程，没有修改代码或提交。本报告的预期来自实际生产分支，不能作为本轮真实通信或飞行结果。

## 1. 可复用夹具与限制

`tests/ros_offline_timing_smoke.py` 已具备真实 ROS 2 本机发布/订阅、唯一源图、IPC STATUS、同步时钟、收集目标/心跳/命令及子进程收尾。它固定 Domain 187、ROS_LOCALHOST_ONLY=1、FastDDS，明确关闭所有动作门，并断言每个目标精确零推力、任何 VehicleCommand 都失败。因此不能只打开配置门就把原脚本称为动力生命周期测试：新场景要明确区分合成消息的非零目标与真实电机执行，保留本机隔离和命令白名单。

`native/tests/test_rc_navigation.cpp` 的 `production_no_vision_rc_config()` / `production_no_vision_rc_1p5m_lifecycle_uses_vertical_only()` 是完整 RC 路线合同来源；`tests/test_stage2_powered_node_equivalence.py:powered_harness()` 是真实 SystemNode + 假 ROS 的 707 状态样本完整路线，已经包含下述接地后事件队列与纯 Runtime 夹具的区别。

## 2. 真实 ROS 话题上人工 RC 准入序列

所有 PX4 消息 timestamp / timestamp_sample 单位微秒，使用当前 ROS 同步墙钟域；raw 不晚于 envelope，必须严格推进且真实接收继续。建议同一发布循环每 50 ms 生成一次当前源戳（20 Hz），`timestamp_sample=timestamp-2000`，不复用已过期测量。不能使用 PX4 boot 微秒直接假冒同步域。

源话题（均 `/fmu/out/`）：

| 消息/话题 | 必需事实 |
| --- | --- |
| VehicleStatus / vehicle_status_v1 | system_id=1、component_id=1；HIL_OFF、pre_flight_checks_pass=true、power_input_valid=true；failsafe=false、failsafe_and_user_took_over=false、无 calibration；can_set_nav_states_mask 包括 STAB |
| VehicleLandDetected / vehicle_land_detected | 初始 landed=true、at_rest=true；ground_contact/maybe_landed 可以同为 true；独立推进 source stamp |
| VehicleLocalPosition / vehicle_local_position | z 初始可取 +2.0 m NED，vz=z_deriv=0；z_valid/v_z_valid=true；z/vz reset 固定；无需 XY/VXY/heading 有效即可走无视觉高度路线 |
| VehicleAttitude / vehicle_attitude | wxyz=[1,0,0,0]，有效近单位四元数；quat_reset 固定；envelope/raw 独立推进 |
| FailsafeFlags / failsafe_flags | 初始所有故障位 false，特别 manual_control_signal_lost/offboard_control_signal_lost/fd_critical_failure=false |
| BatteryStatus / battery_status | connected=true、warning=0、voltage_v 有限正值、remaining 可取1；此源寿命仅1 s |
| ManualControlSetpoint / manual_control_setpoint | valid=true、data_source=SOURCE_RC(1)、roll/pitch/yaw=0、throttle=-1、sticks_moving=false；raw/envelope均推进，至少两次有效接收建立进展 |
| ManualControlSwitches / manual_control_switches | 初始 arm_switch=3、kill_switch=3；mode_slot=6（当前导航档为1，初始必须不是1）；return/loiter/offboard switch=3 或现有合同允许的0；raw/envelope推进 |
| VehicleCommandAck / vehicle_command_ack | 即使正常 RC 起飞不应发Arm/Offboard命令，仍须存在唯一源发布器以满足现有图所有权合同；不得用臆造ACK提前完成命令 |

控制输出接收器（RELIABLE、深度10）必须覆盖 `/fmu/in/vehicle_attitude_setpoint`、`offboard_control_mode`、`vehicle_command`，每个方向不应出现第二个意外发布器；源使用 BEST_EFFORT、深度10 与原脚本一致。`snapshot` 要求 status/land/ack/source 及 command subscriber/owner 等唯一数量，DDS discovery 尚未完成时不能硬编码固定短延迟就授权。

正常顺序应由观察到的阶段推进，而不是只靠 sleep：

1. 未解锁 Stabilized(nav_state=15)、安全地面和RC持续健康。地面静止2 s之后，还要真实新地面样本跨完整证明窗；随后完成至少1.2 s零推力预发送及最低提交数。等 `phase=RC_READY_WAIT_ARM`，不能强行设 armed 绕过 READY。
2. 新 `arm_switch=1` 边沿晚于 READY 源水位；随后 status=ARMED(2)、armed_time=非零稳定值（夹具用10）、latest_arming_reason=2（RC_SWITCH），保持 nav_state=STAB，landed仍true、at_rest可false。状态先到和开关先到均有短窗配对支持，但第一版真实ROS场景建议明确顺序，避免把乱序与主场景混合。
3. 等 `RC_ARMED_WAIT_NAVIGATION`，将新 mode_slot 改为配置导航档1；再提供独立新 status 的 nav_state=OFFBOARD(14)。只有开关变化不等于PX4已经Offboard。进入 `ARMED_GROUND`，初始输出仍精确零推力。
4. 已有 armed dwell 后到 `THRUST_RAMP`。不改变推力配置；hover=.50、reserve=.04、min=.12、max=.80、slew=.40/s。人工路线不允许任何自动 Arm(400,param1=1) 或 Offboard(176) 命令。
5. 合成反馈上升时用 `height=z0-z`，向上速度 `-vz`；测量连续，z_deriv配实际位置差分。landed=false 需要新独立帧，离地探测和运动窗口都合格后到 `ASCEND`，不能只靠时间/推力完成离地。
6. 当前目标1.5 m；轨迹到目标、已证实离地、!landed、实测高度误差≤.15m、速度≤.15m/s连续2s才到 `HEIGHT_HOLD` / `TARGET_SETTLED`。若合成阶跃/突变不符合导数积分，先出现真实反馈保护是合理结果。

## 3. 活动期单一故障的准确预期

以下每一组必须从独立干净进程完成正常准入并到 ASCEND/HEIGHT_HOLD 后注入，避免前一组锁存状态污染下一组。故障发生后继续健康尾帧，验证锁存不被清除。若同时注入多个条件，首因按生产顺序决定，不能期望特定唯一文案。

| 注入 | 预期 phase/reason 与输出 | 必须注意 |
| --- | --- | --- |
| 停止 local 或 attitude 全源发布直到源超时 | 通常 `SAFETY` / `DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE`；后续无新的完整目标/心跳提交 | `health()` 先查 source_clock，再查各 fresh。不能固定期望 Z_VZ_STALE 或 ATTITUDE_STALE；若同步源戳仍新而 raw接收进展停滞，才可能落入对应fresh拒绝。只冻结raw并刷新envelope会永久回退/停滞保护，与纯静默不同 |
| 继续新鲜消息，但 q 非有限/零向量 | `SAFETY` / `ATTITUDE_STALE_OR_INVALID`，无输出 | 排队安全扫描可更早锁存；健康后帧不能冲掉 |
| 活动期 quat_reset_counter+1，q不变或有效改变 | `SAFETY` / `ACTIVE_QUATERNION_RESET`，无新输出 | 有效 q 不证明估计器参考连续；不要把此预期写成已确认误报 |
| 活动期 z_reset_counter 或 vz_reset_counter+1 | `SAFETY` / `ACTIVE_VERTICAL_ESTIMATOR_RESET`，无新输出 | 保持其余状态有效，避免原位置跳变先引入另一原因；z0不能在armed重建 |
| 新 status nav_state 从OFFBOARD14变STAB15，RC其余不变 | `SAFETY` / `OPERATOR_TAKEOVER_LATCHED`；takeover_reason=`PX4_LEFT_OFFBOARD`；无新目标/心跳 | 若同步改mode_slot，先命中显式干预也合理。要测试PX4_LEFT_OFFBOARD，应保持RC开关事实不变 |
| 新 failsafe flags offboard_control_signal_lost=true，其余仍Offboard | `SAFETY` / `OPERATOR_TAKEOVER_LATCHED`；takeover_reason=`OFFBOARD_HEARTBEAT_LOST`；无新输出 | 这是反馈声明已失联，不证明实际ROS心跳接收间隔；只模拟事实 |
| 新 RC sticks_moving=true 或不安全杆量；PX4仍armed+Offboard | 先 `DESCENT`（或原安全恢复阶段），reason常为 `TAKEOVER_REQUEST_PENDING_CONTROLLED_GROUND_RECOVERY`；`m1_operator_release_requested=true`，可继续健康受控下降目标/心跳 | 不能断言立即静默！请求接管≠已观察PX4交接；后续真实离开Offboard才锁存 `OPERATOR_TAKEOVER_LATCHED`，不得自动夺回 |
| 新开关 mode_slot / arm / auxiliary 的非预期变化；仍armed+Offboard | 与上组相同，generation变化被当显式干预 | 可单独测试 mode_slot=6；单测稳定输出时不要每帧变化 switch_changes，否则会伪造连续人工干预 |
| kill_switch=1，或新 status TERMINATION，或fd_critical_failure=true | `SAFETY` / `OPERATOR_TAKEOVER_LATCHED`，takeover_reason=`KILL_OR_TERMINATION`；不继续目标/心跳 | Kill优先级最高；遥测Kill只有可信更新OFF能清，但Core任务故障不靠OFF清除。不得以此验证真实电机停转时延 |
| 只有持续 `abs(vz-z_deriv)>.20m/s`，z与z_deriv积分仍一致 | `vertical_feedback_rate_warning=true`，健康/控制继续，不能独自切Safety | 若z与z_deriv不一致又触发 `VERTICAL_POSITION_RATE_INCONSISTENT` 属另一个真实保护，不能误判速度差诊断变停机 |

对应生产路径：`Core::takeover_cause`、`observe_safety_event`、`Core::step`、`health`，以及 `SystemNode::populate_attitude_feedback`。takeover reason通过 Runtime 的authority JSON字段公开。普通phase reason是锁存首因，后续另查 `rc_first_fault_reason` / `rc_latest_fault_reason` 和 publication_block_reason。

## 4. 安全恢复与正常第二轮任务

Safety后不自动重启进程，也不在空中复位Core。故障在恢复白名单内只是允许**开始证明**，不是已经完成恢复。必须继续发布以下当前安全事实：

- 新 status DISARMED、nav_state=STAB、非Offboard；failsafe/operator_takeover/kill清；preflight、电源和电池有效。
- 新 landed=true、at_rest=true、有效稳定z/vz，abs(vz)≤.10m/s；所有时钟有效，没有源回退/事件溢出或发布完整性锁。
- 新RC arm_switch=3、kill_switch=3、mode_slot有效且≠导航档1；安全中杆/低油门、辅助开关安全，RC来源和独立开关均新鲜推进。
- 首先重获节点2 s未解锁静止；随后恢复窗口还需独立新status≥2、local≥3、land≥3，新的land实际接收跨ground_seconds=2s、位置/导数窗口检查通过，所有源戳晚于故障水位。冻结land后只空跑tick不能凑满。
- 未决176/400/21必须满足原终态规则，不能用迟到ACK认领新事务；publication_fault/time_regression/未知故障不能靠拨开关恢复。

`rc_reprepare_block_reason`、`rc_recovery_block_reason`、`rc_recovery_elapsed_s`、各sample count和fault watermarks应解释当前等待原因。若服务正shutdown，`RC_SHUTDOWN_IN_PROGRESS`是合理阻塞；不能修改成自动再起飞。

重新准备成功输出 `kind=rc_safe_reprepare`，Core重建但全局ACK/提交历史保留；只到新的地面预发送/`RC_READY_WAIT_ARM`，必须再次人工arm边沿+新Offboard证据。不能断言一恢复就非零推力。

正常降落先native LAND，测量下降且接地新证据充分后计划Disarm；只有真正新DISARMED status才能完成。纯Runtime夹具在SafeDisarmed后直接喂STAB可到Idle；**真实SystemNode**接收外部开关/模式离开会保留authority/status事件，通常先 `SAFETY/OPERATOR_TAKEOVER_LATCHED`，再新地面证明回准备。707帧powered node夹具明确断言后者，真实ROS测试应沿此合同，不照抄纯Core瞬时Idle期待。

## 5. native STOP、LAND 和 Supervisor 命令不能混淆

- 原生IPC `LAND` 调用 `Core::descend`，不等于关节点；健康活动任务沿原受控下降。没有新接地证据不能自动上锁。
- 原生IPC `STOP` 在姿态profile要求新鲜status+land、已DISARMED、landed、at_rest，否则拒绝 `STOP_REQUIRES_CONFIRMED_DISARMED_GROUND_STATE`。接受后才由SystemNode锁shutdown；它不是飞行中急停命令。
- `navigation.py land/stop` 都映射 `sudo systemctl stop fc01-navigation.service`；Supervisor收到停止后 `NodeStop.request()` 向自己管理的原生进程发一次SIGTERM。原生信号处理只置停止请求；tick内 `begin_shutdown()`先锁shutdown再调用LAND。Agent保持可用，等待原生安全退出；不是IPC STOP，也不是发送Kill或直接强杀。
- shutdown期间禁止下一轮。节点通常只有新鲜未解锁接地且任务不active才退出；60s到期也不截断仍armed+Offboard的健康受控恢复。严重故障路径最终依赖PX4/操作员，测试不应无限等待而省略说明。

## 6. 建议保留的证据字段

每个场景至少保存kind/state/reply原文、接收monotonic、源消息timestamp/timestamp_sample以及目标/心跳/command真实订阅回调时间；phase/reason/takeover_reason，armed/nav_state/landed/at_rest，source_clock_valid/validated、各fresh、publication_stage/decision/target_committed/heartbeat_committed/sink_exception；height_reference_z0、height_sample_z、relative_height_m、velocity_ned[2]、reset counters；takeoff_stage/liftoff_*/target_height_settled；thrust_body、planned/committed推力；rc_*首因/恢复窗口及rc_safe_reprepare事件。

本地target/heartbeat commit和模拟订阅收到都不能写成PX4采用。本阶段若无硬件Position日志，不以合成轨迹算真实超调、保持误差或水平漂移结论。真实相同航次仍需ULog与companion日志可靠对时。
