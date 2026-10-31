# 视觉导航接入与竞赛运动控制设计

依据用户本轮需求和“大学生竞赛，非武器制导”的用途澄清。新增通用定点、点目标和连续运动输入；不增加识别/攻击/武器接口。用户要求继续实施，未知真实接口采用明确标注的内部合同。

## 基线与方法

本目录原来没有 Git 元数据。`navigation-repair-20261031` / `6b440ab` 保存当前导航修复源码快照；开发分支为 `feature/vision-navigation`。原导航档位 1、手动解锁、0.5m、20Hz、30s 任务时限及所有已有推力/PID/保护参数继续作为基线。

审查确认 attitude_altitude 只有 Z/Vz 闭环、水平姿态与锁定 yaw，无 XY 闭环。采用伴随端位置 P→速度 P→水平加速度→受限四元数外环，继续使用原垂直控制器和唯一姿态/推力双 sink；不切换到旧 trajectory 发布分支。另一种方案是把视觉送 PX4 EKF 后使用 PX4 位置控制，但依赖飞控参数和实机融合验收，不适合本次开发范围。

## 临时内部合同 V1

这不是新版相机合同。默认内部话题 `/fc01/vision/state` 和 `/fc01/navigation/command`，可配置；标准 Odometry 通过独立适配器转换。旧 `/odometry`、`/openvins/odom`、`/imu`、`/odometry_rect` 不作为默认；`/odomtry` 保留原拼写待核实。

内部世界坐标 `fc01_vision_ned`（x/y/z，米；速度米/秒），机体系 `base_link_frd`（前/右/下）。四元数 Hamilton w,x,y,z，表示 FRD→内部 NED；角速度在 FRD、rad/s，来源字符串必须随有效字段提供。内部 NED 必须经过与 PX4 航向的标定/对齐；不从未知相机坐标自动推断。

状态和目标分别有版本、source_id、严格推进 sequence、ROS/system 测量 stamp、接收端 steady 时间、reset_counter。消息丢字段用 validity mask 与 NaN/null 表示，不用默认零宣称有效。状态包含 position[3]、velocity[3]、attitude[4]、angular_velocity[3]、pose/twist covariance[36]、initialized/tracking/calibration_valid、quality。控制要求位置、速度、姿态、quality 有效且已初始化/跟踪/标定；角速度和协方差可缺失并明确记录。角速度不替代 PX4 姿态内环。

命令模式 Disabled/Hold/Cruise/Follow/Land，和状态分别计时/去重。Cruise 为绝对内部 NED 点目标和可选绝对 yaw；不把相对位移自动当绝对坐标。Hold 在模式进入时捕获当前位置。Follow 四路只表示运动请求，不提供任何 ARM/Offboard 或权限租约。

## 四路映射与新增候选参数

- roll ±1 → 机体左右速度 ±0.5m/s，正为右。
- pitch ±1 → 机体前后速度 ±0.5m/s，正为前。
- yaw ±1 → 向右偏航角速度 ±0.35rad/s。
- throttle ±1 → 升降速度 ±0.15m/s，正为上；积分形成有界高度目标。0 为零升降速度请求，保持累积高度目标，原 Z/Vz 闭环仍计算推力。
- XY 位置增益 0.8/s，速度增益 1.2/s；水平加速度 0.6m/s²、运动目标倾角 5°、活动半径 2m；都为单独的仿真候选参数，不是实机标定值。新增限幅还受原 max_tilt、max_height、climb/descent 限制，不能扩大旧包络。
- 状态 TTL 0.25s、命令 TTL 0.30s 同时限制源测量年龄与 steady 接收年龄；最大到达传输延迟 0.20s、未来容差 0.05s、最小 quality 0.5。传输延迟消耗 TTL，不由收到消息重新续命。这些仅为临时内部合同的开发值，视觉频率不是控制 20Hz 的推论。

所有运动只在已有 RC 授权完成并实际进入 HeightHold 后开始。起飞/降落仍由原主线控制。视觉 z 用于把点目标误差转换为原垂直控制器的高度目标，垂直快速反馈仍用 PX4 Z/Vz；视觉 x/y/vx/vy 实际参与水平闭环。视觉姿态用于机体速度变换，yaw 目标受限速；没有额外角速度内环，也不注入 PX4 EKF。

## 失效与最终发布

首次有效命令后，状态/命令过期、非法值、重置/来源/坐标变化、跳变、Disabled/Land 均清理运动目标并锁存原生受控降落；新数据恢复不自动退出降落。未激活时缺少视觉输入仅等待，保留原起飞/定高行为。遥控接管、Kill、PX4 fail-safe、mode-out 始终优先；视觉不重入 Offboard。

仅有效运动许可可在 HeightHold 放行受限倾斜/yaw，最终 sink 再验证时效、当前输入、原权限与输出四元数；未启用视觉时保留原 level guard。下降冻结最后实际提交 yaw，不能突然转回初始航向。目标降低也必须沿原 max_descent 双向限速。原生健康/Kill/部分 sink 错误继续原硬故障；纯任务输入过期且尚未提交任何 sink 时请求原生下降，不当作控制器硬错误。

## 核心代码接口（实现约定）

`vision_navigation.hpp`，namespace `fc01::vision`，只依赖标准库和 attitude_reference：

- `using Vec3=std::array<double,3>`，`enum class Mode {Disabled=0,Hold=1,Cruise=2,Follow=3,Land=4}`。
- `Config`：`enabled` 默认 false；上述参数用 `state_timeout,command_timeout,max_measurement_delay,future_tolerance,position_gain,velocity_gain,max_speed,max_acceleration,max_tilt_deg,max_yaw_rate,max_vertical_speed,max_radius,max_height,min_quality,max_position_jump,max_velocity_jump`，跳变默认 0.75m/1m/s。`validate()`。
- `State`：`sequence,stamp_ns` uint64，`received_at` double（默认 -1），`reset_counter` uint32，`frame_id,body_frame_id,source_id,angular_velocity_source` 字符串；`initialized,tracking,calibration_valid` bool；`valid_fields` uint16，位 Position=1/Velocity=2/Attitude=4/AngularVelocity=8/PoseCovariance=16/TwistCovariance=32/Quality=64；`position,velocity,angular_velocity` Vec3；`attitude` Quaternion；`pose_covariance,twist_covariance` array36；`quality` double。测量数组默认 NaN。
- `Command`：`sequence,stamp_ns,received_at,reset_counter,source_id,frame_id`；`Mode mode`；`valid,position_valid,yaw_valid,sticks_valid` bool；`position` Vec3、`yaw` double、`sticks` array4（roll/pitch/yaw/throttle），缺值 NaN。
- `Input`：`State state; Command command; uint64_t ros_now_ns{}; size_t state_publishers{},command_publishers{}`；另有仅本地的 `uint64_t revocation_generation{}`，不是 ROS 字段。Inbox 在身份/epoch/frame 变化或可信撤销时推进此代次，已激活 Controller 据此锁存降落，避免重启低序号及 A→B→A 被过滤；尚未激活时初始化恢复可正常开始。
- `Result`：`active,request_land` bool；`Mode mode`；`reason` string；`q` Quaternion；`target_height,yaw` double；`velocity_target,acceleration_target` Vec3；`state_sequence,command_sequence` uint64；`reset_counter` uint32。
- `Controller(Config)`；`Result step(const Input&,double now,double px4_height,double px4_yaw,double dt)`；`bool current(const Input&,double now)const`（包括已接受 plan 的输入身份及固定时效）；`const Result& last()const`；`void stop()`。不生成飞行命令或推力。
- `Inbox`：`bool state(State value,double received_at)` / `bool command(Command value,double received_at)`；`Input snapshot(uint64_t ros_now_ns,size_t state_publishers,size_t command_publishers)const`。重复/倒序不刷新接收时间；新无效消息会撤销有效状态；reset 变化保留交给控制器锁存失效。

ROS `.msg` 显式对应上述字段（stamp 用 builtin_interfaces/Time，四元数仍显式 wxyz 数组）。标准 nav_msgs/Odometry 适配需匹配同时间戳的显式 VisionHealth 元数据，未确认外参/初始化/质量不得生成有效状态。非零杠杆臂无角速度不能把速度默认有效；协方差未变换则撤销 covariance validity。

## 验证与交付

独立 preview/replay 进程只输出模拟姿态/推力/最终边界决定，不建立任何 `/fmu/in/**` publisher。测试通过真实 Runtime、RC 启动、Core、final boundary；模拟动力学明确为理想模型，不是飞行验收。覆盖点目标、四路、状态扰动、源时钟/TTL/重复、frame/extrinsic/reset、模式切换、RC 接管及 partial sinks，保留完整旧回归。真实 Humble+原 FC01 SDK 构建与自定义接口生成单独验证。

交付字段表、操作命令、候选参数用途、缺失真实设备资料、修改范围、基线提交/开发提交与合并步骤；不切服务、不改飞控参数、不做动力测试。
