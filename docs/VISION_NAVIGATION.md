# 视觉导航内部合同、离线验收与集成

2026-11-03 的起飞证据、1.5 m 平台、重新准备及通信审查见 [当前修复记录](TAKEOFF_AUDIT_20261103.md)。下面单独指定的回放位置和外环限幅不代表默认起飞高度。

本次增加伴随端通用定点、点目标和连续运动输入。新增视觉控制默认关闭。真实设备消息、外参和时钟尚未确认时，不能把下面的内部合同当作相机驱动接口。原导航档位 1、遥控手动解锁、1.5 m 默认起飞平台、20 Hz、30 s 任务上限以及原 Z/Vz 控制器继续生效。

设计依据见 [设计稿](superpowers/specs/2026-10-31-vision-navigation-design.md)。离线程序只使用合成 PX4/RC 状态；即使输入标记为 `recorded`，回放中的 RC 启动、PX4 反馈和发布接收端仍然是虚拟对象。它不会建立 ROS 节点或 `/fmu/in/**` publisher，不调用 service、agent、安装程序或飞控参数接口。

## 数据合同 V1

内部世界系为 `fc01_vision_ned`：x 内部北向参考轴、y 内部东向参考轴、z 向下，单位 m、m/s。机体系为 `base_link_frd`：前、右、下。内部 NED 还必须通过显式标定与 PX4 世界系、航向对齐；不能仅凭名字推断相机外参。四元数为 Hamilton `[w,x,y,z]`，表示 FRD → 内部 NED；角速度在 FRD，单位 rad/s。

`Input.revocation_generation` 是 Inbox 在接收进程内维护的撤销代次，不在 ROS 消息中传输。源/坐标/重置周期变化或健康/validity 撤销会使旧运动计划失效，包括同一控制周期内 A→B→A 的切换以及源/周期重启后的低序号消息；因此不能只用最终 source 字符串和递增序号判断身份。未开始运动时允许初始化输入建立新计划，已发生的任务失效不会因新健康数据恢复而重新启动。

默认内部话题是 `/fc01/vision/state`、`/fc01/navigation/command`，可配置。标准 Odometry 必须经过独立适配器与同时间戳的显式健康元数据匹配。旧 `/odometry`、`/openvins/odom`、`/imu`、`/odometry_rect` 不自动成为数据源；旧 `/odomtry` 的拼写和真实含义仍待设备侧确认。

### 状态字段

| 字段 | 类型/单位 | 约束 |
| --- | --- | --- |
| `sequence` | uint64 | 非零推进；普通重复/倒序消息不续 TTL，源/周期变化及显式撤销另行处理 |
| `stamp_ns` | uint64，ROS/system ns | 测量时间，与 `ros_now_ns` 属于同一源时钟域 |
| `reset_counter` | uint32 | 状态与命令须处于同一标定/估计周期 |
| `received_at` | 接收端 steady s | 仅进程内部使用；TTL 同时检查 steady 接收年龄及当前源测量年龄 |
| `source_id` | string | 非空；运动启动后变化触发降落 |
| `frame_id`, `body_frame_id` | string | 必须是明确的内部 NED、FRD 系 |
| `initialized`, `tracking`, `calibration_valid` | bool | 三者均须成立 |
| `valid_fields` | uint16 位掩码 | 字段有效性的显式声明 |
| `position[3]` | m，NED | 控制必需 |
| `velocity[3]` | m/s，NED | 控制必需；不是由缺失值补零 |
| `attitude[4]` | wxyz，FRD→NED | 控制必需；范数须有效 |
| `quality` | 0…1 | 控制必需，达到最小质量 |
| `angular_velocity[3]`, `angular_velocity_source` | rad/s，FRD / string | 可缺失；声明有效时必须有来源 |
| `pose_covariance[36]`, `twist_covariance[36]` | 行主序 6×6 | 可缺失；未完成坐标变换须撤销 validity |

掩码值：Position=1、Velocity=2、Attitude=4、AngularVelocity=8、PoseCovariance=16、TwistCovariance=32、Quality=64。最小运动状态是 `1+2+4+64=71`。缺失数组/标量使用 NaN（JSONL 为 `null`）并撤销对应有效位。不得用默认单位四元数、零速度或零协方差冒充测量。非零杆臂的速度转换需要角速度，缺失时不能声明转换后的速度有效。

### 命令字段与模式

命令独立包含 `sequence, stamp_ns, received_at, reset_counter, source_id, frame_id`，独立计时和去重；另外有 `mode, valid, position_valid, yaw_valid, sticks_valid, position[3], yaw, sticks[4]`。

| 模式 | 含义 |
| --- | --- |
| Disabled=0 | 清理运动目标；已经开始运动则锁存原生受控降落 |
| Hold=1 | 进入模式时捕获当前位置，后续位置/速度扰动进入闭环 |
| Cruise=2 | 绝对内部 NED 点 `position`，可选绝对 yaw（rad） |
| Follow=3 | 四路运动请求，经视觉姿态转换成世界速度 |
| Land=4 | 请求原生受控降落；不提供解锁/Offboard 权限 |

Follow 四路为 `[roll,pitch,yaw,throttle]`，每路范围 `[-1,1]`。正 roll 向右，正 pitch 向前，正 yaw 向右转，正 throttle 上升。中间油门为零升降速度，保持积分后的高度目标，Z/Vz 控制器仍持续计算推力。命令目标是绝对坐标，不自动把相对位移转换成绝对点。

### 新增外环仿真候选值

| 参数 | 候选值 | 用途 |
| --- | --- | --- |
| `position_gain`, `velocity_gain` | 0.8/s，1.2/s | XY 位置 P → 速度 P → 水平加速度 |
| `max_speed`, `max_acceleration` | 0.5 m/s，0.6 m/s² | 运动速度与水平加速度限制 |
| `max_tilt_deg`, `max_radius` | 5°，2 m | 姿态目标与活动范围限制 |
| `max_yaw_rate` | 0.35 rad/s | yaw 变化率限制 |
| `max_vertical_speed` | 0.15 m/s | Follow 高度积分限制 |
| `state_timeout`, `command_timeout` | 0.25 s，0.30 s | 分别同时限制当前源测量年龄和 steady 接收年龄；传输延迟消耗 TTL |
| `max_measurement_delay`, `future_tolerance` | 0.20 s，0.05 s | 前者只限制到达时的传输年龄，后者限制未来时钟偏差 |
| `min_quality` | 0.5 | 最小可用质量 |
| `max_position_jump`, `max_velocity_jump` | 0.75 m，1 m/s | 相邻状态跳变拒绝 |

这些是内部合同开发和仿真候选，不能作为实机标定结果。倾角、高度、升降目标还受到原 `max_tilt_deg`、`max_height`、`max_climb/max_descent` 限制；新增限制不能扩大原包络。

## 控制与权限

只有完整 RC 授权并实际进入 `HEIGHT_HOLD` 后才使用视觉运动。视觉 x/y/vx/vy 实际进入水平闭环，视觉姿态参与机体系速度转换；视觉 z 只将目标误差换算为原控制器的高度目标。快速垂直反馈仍是 PX4 Z/Vz，推力仍由原 `VerticalController` 计算；没有额外角速度内环或 PX4 EKF 注入。

缺少输入且尚未开始视觉运动时继续原起飞/定高等待。首次有效运动后，状态/命令过期、非法测量、重置、来源/坐标变化、跳变、Disabled/Land 均锁存原生降落；新数据恢复不会重新开始运动。RC 接管、Kill、PX4 fail-safe、实际 mode-out 保持原优先级。下降沿原 max_descent 限制高度目标，并冻结最后实际提交的 yaw。

所有输出通过真实 `Runtime → Core → publish_attitude_at_boundary`。最终边界再次检查当前反馈、原权限、视觉输入身份/时效和输出四元数。回放的两个接收 lambda 只记录实际调用，不发送网络或硬件数据。首次提交前的策略拒绝不会调用任一 sink；target 已提交后的重采样失败保留原 partial-sink 故障，不伪装为未发送。

## 运行离线工具

依赖 Python 3、PyYAML、g++（C++17），不依赖 ROS。仓库已有 PyYAML 用于参数文件。首次运行会编译 host driver，产物只在仓库 `.superpowers/vision_replay/<源码摘要>/` 缓存；后续源码变化会生成新的缓存。

```bash
python3 vision_replay.py --demo cruise > /tmp/vision_cruise.jsonl
python3 vision_replay.py --demo height > /tmp/vision_height.jsonl
python3 vision_replay.py tests/fixtures/vision/follow_land.jsonl
python3 -m unittest tests.test_vision_replay -v
```

`--config PATH` 可选择另一份 `fc01_system.ros__parameters` YAML。默认直接读取 `native/config/attitude_altitude.yaml`；从实际 `attitude_ros_parameters.hpp` 导出 ROS 参数→C++ 成员映射，再将 YAML 值送入真实 `Config::validate()`。因此当前 hover=0.50、reserve=0.04、thrust_max=0.80、height_gain=0.50、velocity_gain=0.15、integral_gain=0.03 等参数进入真实控制器，工具没有替换一套简化 PID。模拟会在当前进程中启用 RC 路由及参数确认、关闭自动 Arm/bench/autonomous 权限，固定导航档位 1，起飞平台读取当前 `rc_target_height`（默认 1.5 m）；不写回 YAML 或改变系统配置。

启动时，`VisionFlight` 先模拟合法新鲜 RC/预流、真实双 sink 提交、手动解锁开关与 PX4 armed 证据、导航开关与 PX4 Offboard 证据，再走原 ground dwell/ramp/liftoff/ascent/settle。它不会调用 Core 的阶段跳转或注入权限。输出中的 `arm_commands`、`mode_commands` 应始终为零。

`--demo cruise` 是 9 s 理想闭环：目标 `[0.8,0.4,-1.5]` m（默认平台配置），植物只从最后实际提交的 q/thrust 积分出速度和位置，再经 Inbox 回到真实 Runtime。`--demo height` 把高度目标改为平台上方 0.4 m（默认 1.9 m），验证原垂直环的推力响应。模型假设姿态瞬时跟踪、小阻尼、hover 对应重力，无真实执行器、延迟、噪声或气动模型；误差下降只能证明代码闭环，不是飞行性能验收。

### JSONL 会话格式

首行必须声明版本与数据来源，后续每行一个 `tick`。严格 JSON 不允许 NaN/Infinity、重复字段或未知字段；缺测用 `null`。数组尺寸、bool/uint 类型和 dt 在启动 C++ 前完整验证。非法控制测量可以用 `null` 或无效掩码输入，由真实控制器决定等待/降落。

```json
{"schema_version":1,"kind":"session","dataset_kind":"synthetic","description":"明确标注合成来源"}
{"kind":"tick","dt":0.05,"ros_now_ns":1050000000,"state":{"sequence":1,"stamp_ns":1050000000,"source_id":"example_vision","frame_id":"fc01_vision_ned","body_frame_id":"base_link_frd","initialized":true,"tracking":true,"calibration_valid":true,"valid_fields":71,"position":[0,0,-0.5],"velocity":[0,0,0],"attitude":[1,0,0,0],"quality":1},"command":{"sequence":1,"stamp_ns":1050000000,"source_id":"example_plan","frame_id":"fc01_vision_ned","mode":"Cruise","valid":true,"position_valid":true,"position":[0.5,0.2,-0.5]}}
```

可选 session `candidate_parameters` 只接受上表外环数值字段，不能覆盖原 PID/推力/RC 权限。

| tick 字段 | 行为 |
| --- | --- |
| `dt` | steady 时间增量，默认 0.05 s；大停顿仍受 Core 的原 stall guard |
| `ros_now_ns` | 显式测量时钟；默认 0 表示未知，不能构造有效运动输入 |
| `state`, `command` | 独立新消息；省略表示该源没有新消息，不刷新 TTL |
| `received_age_seconds` | state/command 子字段，默认 0；重建该消息已在接收端停留的时间 |
| `state_publishers`, `command_publishers` | 默认 1；零或多源交给真实控制器拒绝 |
| `px4.height`, `px4.up_speed`, `px4.yaw` | 可选合成 PX4 观测，单位 m、m/s（正上）、rad；省略沿用启动后的状态 |

`sequence`、`stamp_ns` 缺失按无效值 0 交给原生 Inbox，不能创建新有效消息。状态数值数组和 quality 缺失为 NaN；初始化/validity 默认为 false/0。整个会话的 `schema_version=1` 同时声明内部状态和命令合同版本。

输出每行也是 JSON，包含输入与已接受的 seq、active/原因/native_reason、速度/加速度目标、真实 q/thrust、高度目标、phase、最终 allowed/boundary_reason、两个 sink 是否实际提交及累计计数。`simulation=true` 始终成立，`dataset_kind` 仅说明载入的数据原来是合成还是记录。`q/thrust` 是 Core 算出的真实输出值；`allowed=false` 时不是已发送命令。数值缺失输出 null。

### 真实录制与模拟的区别

仓库 `tests/fixtures/vision/*.jsonl` 全部是手写合成回归数据，均有 `dataset_kind=synthetic` 标识，没有真实相机/飞行录制文件。真正的记录应使用 `dataset_kind=recorded` 并在 `description` 记录设备、软件版本、坐标标定和采样条件。需从测量原始时间、接收 steady 时间、有效位、健康信息与独立命令序号重建 JSONL；不要按回放节拍重新给旧测量盖章。外部录制格式必须先显式转换为本合同，工具不会猜测坐标、质量或时间源。

当前回放入口会先生成虚拟 RC/定高启动；后续只有显式 `px4` 字段才能改变该回放的 PX4 垂直观测。仅播放视觉记录而没有同步 PX4 观测，不会成为真实动力学重现。理想植物仅在 `--demo` 场景启用。

## 离线验收与实机待确认

自动回归覆盖：水平点误差下降、位置/速度改变姿态、高度目标改变原控制器推力、Follow 四路、缺失输入等待、非法字段/坐标、状态与命令各自的 TTL 失效与锁存、重复状态不续 TTL、模式切换，以及真实 RC/双 sink 路径。`native/tests/test_visual_runtime.cpp` 已覆盖 RC 手动启动、位置/速度对姿态及高度目标对原推力环的影响、降低高度的下降限速、冻结最后实际提交 yaw、输入 TTL 触发原生降落、target 提交前的软任务退出、部分 sink 后的硬故障，以及 RC 杆量/Kill/mode-out 优先；核心/边界测试同时覆盖重置/跳变、最终时效竞态和原有回归。具体运行数量与结果以本轮验收记录为准。

完整代码验收至少执行仓库 Python 全回归、原生 C++ 全回归及 ROS2 Humble + 原 FC01 SDK 的实际接口生成/构建。host driver 能证明原生控制链执行，不能证明 ROS 消息生成或真实设备 QoS。

仍需设备资料：相机/VIO 消息定义与 topic、频率/延迟/测量时钟、初始化和 tracking 语义、质量与协方差、世界/机体系定义、相机→机体外参及杆臂、PX4 航向对齐、reset_counter 来源、真实 RC 接管与 fail-safe 实机记录。资料与硬件验收完成前，不能据此开启实机视觉运动。

## 修改与合并

修复快照基线为 `navigation-repair-20261031`，提交 `6b440ab`；开发分支为 `feature/vision-navigation`。开发提交号以交付时的 `git log -1 --oneline feature/vision-navigation` 为准，不能把尚未生成的提交号写成已存在。

改动范围包括原生视觉合同/外环、Core/Runtime/最终边界连接、ROS 内部消息与 Odometry 适配、默认关闭的 ROS 参数、host replay/fixtures/测试与本文。原控制参数 YAML 的 PID/推力保护值作为既有基线使用；系统 service、飞控参数及硬件状态不属于本次交付。

完成全部测试并审阅开发 diff 后，在目标修复分支合并：

```bash
git status --short
git log -1 --oneline feature/vision-navigation
git diff navigation-repair-20261031...feature/vision-navigation
git switch navigation-repair-20261031
git merge --no-ff feature/vision-navigation
```

若目标分支继续有修复，先在独立工作目录处理冲突并重跑完整回归，重点保留 RC 手动解锁、新导航开关证据、原生下降以及最终双 sink 的授权条件。代码合并与设备安装/服务切换是独立动作；本文命令只合并源码。

## ROS 2 接口、构建与候选配置

本节命令供代码审阅、独立构建和模拟话题检查使用。默认生产 Node 的 `visual_navigation_enabled=false`，仓库 `vision_settings.json` 也默认关闭。下面不启动生产 `fc01_system_node`、Agent 或服务，不修改飞控参数；示例发布程序只写 `/fc01_demo/**`，使用明确的合成数据与假标定。

### 依赖与本机 SDK 构建

ROS 部分需要 Humble、ament_cmake、rclcpp、rclpy、builtin_interfaces、nav_msgs、rosidl_default_generators/runtime，以及 ros2run/ros2topic/ros2interface 命令。还需要 CMake、C++17 编译器、Python 3 和 PyYAML。`px4_msgs` 使用原 FC01 SDK 固定版本，不能随意替换为最新消息；原编译期版本断言继续生效。本机已有这些 ROS 依赖。

本次本机 SDK 是 `/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632`。单包 CMake 构建可复用该消息环境，构建与安装产物单独放在 `/tmp`：

```bash
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/local_setup.bash
cmake -S native/src/jikl_fc01_system -B /tmp/fc01_vision_ros_20261031 \
  -DCMAKE_INSTALL_PREFIX=/tmp/fc01_vision_ros_20261031/install \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -Dpx4_msgs_DIR=/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/cmake
cmake --build /tmp/fc01_vision_ros_20261031 --parallel 2
cmake --install /tmp/fc01_vision_ros_20261031
source /tmp/fc01_vision_ros_20261031/install/share/jikl_fc01_system/local_setup.bash
```

单包 CMake 安装提供上面的包级 `share/jikl_fc01_system/local_setup.bash`，没有 colcon 聚合的 `install/local_setup.bash`。本包为可执行文件和生成的 typesupport 库安装 `$ORIGIN;$ORIGIN/..` RPATH：可执行文件位于 `lib/jikl_fc01_system`，自身消息库及动态加载后端位于 `lib`。安装启动路径可以找到本包库；无需手工修改全局 `LD_LIBRARY_PATH`。

只读检查库依赖、生成消息与参数：

```bash
ldd /tmp/fc01_vision_ros_20261031/install/lib/jikl_fc01_system/fc01_system_node
ros2 interface show jikl_fc01_system/msg/VisionState
ros2 interface show jikl_fc01_system/msg/NavigationCommand
ros2 interface show jikl_fc01_system/msg/VisionHealth
ros2 interface show jikl_fc01_system/msg/AttitudeThrustPreview
```

`ldd` 应解析本包 `libjikl_fc01_system__rosidl_typesupport_cpp.so`，不能出现 `not found`。上述检查与真实 DDS 接收、设备标定及飞行验收分别记录。

### 设置文件与只读 YAML 检查

仓库默认示例：

```json
{
  "enabled": false,
  "state_topic": "/fc01/vision/state",
  "command_topic": "/fc01/navigation/command",
  "state_reliable": false,
  "command_reliable": true,
  "qos_depth": 5,
  "parameters": {}
}
```

`parameters` 只接受本文新增外环参数名，例如 `{"max_speed":0.3}`；不能覆盖原 PID、推力或权限字段。对应 Node 参数为 `visual_state_topic`、`visual_command_topic`、`visual_state_reliable`、`visual_command_reliable`、`visual_qos_depth` 和 `visual_<外环参数名>`。state 默认 best-effort、command 默认 reliable，均 volatile；深度可配。实际 QoS 必须与源兼容。

只导出候选 YAML，原设置与原服务均不改变：

```bash
python3 vision_config.py --output /tmp/fc01_vision_candidate_off.yaml
python3 vision_config.py --output /tmp/fc01_vision_candidate_on.yaml --enable-vision
ros2 run jikl_fc01_system fc01_config_check --ros-args \
  --params-file /tmp/fc01_vision_candidate_on.yaml
```

`--enable-vision` 仅在导出的候选中置开关，保留原 `vision_settings.json`；不安装、不切服务，也不进行硬件验证。`fc01_config_check` 只创建参数解析节点并运行原 C++ `Config::validate()`，不创建 `/fmu/in/**` publisher。通过表示 YAML/参数约束成立，不表示相机时钟、外参、RC 接管、飞控或运动性能已验证。

### Odometry adapter 的完整显式参数示例

先在参与演示的每个终端 source 上面的 Humble、原 SDK、本包包级环境，再选择隔离演示环境：

```bash
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
```

以下标定 id、单位外参和 alignment 全部是**模拟专用假标定**。外部 topic 必须显式提供，程序不默认使用旧 `/odometry`、`/openvins/odom`、`/imu` 或 `/odometry_rect`；`/odomtry` 保留原拼写，仍待设备确认。

```bash
ros2 run jikl_fc01_system vision_odometry_adapter.py --ros-args \
  -p use_sim_time:=false \
  -p input_topic:=/fc01_demo/raw/odom \
  -p health_topic:=/fc01_demo/raw/health \
  -p output_topic:=/fc01_demo/vision/state \
  -p source_id:=doc_synthetic_vio_session_1 \
  -p input_frame_id:=demo_world_enu \
  -p input_body_frame_id:=demo_sensor_flu \
  -p world_convention:=ENU \
  -p body_convention:=FLU \
  -p body_from_sensor_wxyz:='[1.0, 0.0, 0.0, 0.0]' \
  -p sensor_position_body_frd:='[0.0, 0.0, 0.0]' \
  -p world_alignment_wxyz:='[1.0, 0.0, 0.0, 0.0]' \
  -p world_translation_ned:='[0.0, 0.0, 0.0]' \
  -p extrinsics_id:=DEMO_IDENTITY_MOUNT_NOT_CALIBRATED \
  -p alignment_id:=DEMO_ENU_NED_NOT_CALIBRATED \
  -p qos_depth:=5 \
  -p odometry_reliable:=false \
  -p health_reliable:=true \
  -p output_reliable:=false \
  -p pair_timeout:=0.05 \
  -p max_pending:=32
```

`world_alignment_wxyz` 作用于常规 ENU→NED 转换之后；`body_from_sensor_wxyz` 把传感器 FRD 旋转到机体 FRD，`sensor_position_body_frd` 是传感器原点在机体 FRD 的位置。相机 optical 坐标不能填 FLU 名称后直接使用，需明确前置变换。非零杆臂缺角速度时速度位撤销；首版未变换 covariance，两个 covariance 位均清除。

adapter 在每次输入 callback 和定时检查中读取**原始 Odometry 与原始 Health 各自的真实 publisher 数量**，两者必须分别恰为 1。零源/多源会清配对缓存、发布显式无效 canonical state；只看 canonical state 的一个 publisher 不足以建立上游唯一性。恢复单源后必须重新形成完整配对，不能借用源冲突前缓存的 health。DDS 发现计数是本地当前图观测，不是发布者认证。

有效 state 的 stamp 始终是原始测量 stamp；Health 必须精确匹配。缺 metadata 超过 `pair_timeout` 也发布无效状态，不保留上个样本的健康标志。源冲突等撤销事件的 stamp 是本地 ROS/system 观测时间，`valid_fields=0`、initialized/tracking/calibration=false、测量均 NaN；这个时间只定位撤销事件，不表示新测到了姿态或位置。wire `sequence` 是 adapter 本地单调序号，包含撤销事件，区别于原始 `VisionHealth.sequence`；日志分别打印 `canonical_sequence` 与 `raw_health_sequence`。源恢复不会解除控制器已锁存的任务降落。

### VisionHealth 字段表

所有确认 bool 默认 false。数值默认零也没有有效性，必须由对应 presence/quality/reset 确认。只有明确的模拟源才可在演示中把假标定标为已满足。

| 字段 | 合同 |
| --- | --- |
| `schema_version` | 必须为 1 |
| `stamp` | builtin_interfaces/Time，与这条原始 Odometry.header.stamp 精确相同 |
| `sequence` | 原始测量的非零源序号，与 adapter 输出本地序号分开 |
| `source_id` | 必须匹配配置，建议包含源会话标识 |
| `frame_id`, `body_frame_id` | 必须分别匹配原始 header.frame_id、child_frame_id 及配置 |
| `reset_counter`, `reset_valid` | 明确估计/标定周期及重置语义；未知不能填零宣称有效 |
| `initialized`, `tracking`, `calibration_valid` | 明确初始化完成、正在跟踪及标定成立 |
| `frame_alignment_valid`, `alignment_id` | 已建立内部世界系/PX4 航向对齐，id 匹配配置 |
| `time_sync_valid` | sample 已处于明确的 ROS/system 时间域；到达时间不能冒充 sample |
| `extrinsics_valid`, `extrinsics_id` | 完整旋转/平移外参已确认，id 匹配配置 |
| `metric_scale_valid` | 位置/速度公制尺度已确认 |
| `position_valid`, `velocity_valid`, `attitude_valid` | 分别确认原始测量字段存在且有效 |
| `angular_velocity_valid`, `angular_velocity_source` | 角速度存在且明确来源；来源为空不得设有效位 |
| `quality_valid`, `quality` | 明确内部归一化 0…1 质量；未知不采用默认零推断 |

### 最小模拟发布示例

下面两个例子是替代路径；不要同时对同一 canonical state topic 建立两个 publisher。每个程序仅发布模拟数据，不运行生产控制节点。

直接验证内部 State/Command 消息格式，可在已 source 的独立演示终端运行：

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from jikl_fc01_system.msg import VisionState, NavigationCommand
rclpy.init()
node = Node('doc_synthetic_internal_source')
state_pub = node.create_publisher(VisionState, '/fc01_demo/vision/state',
    QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
command_pub = node.create_publisher(NavigationCommand, '/fc01_demo/navigation/command', 5)
sequence = 0

def publish():
    global sequence
    sequence += 1
    stamp = node.get_clock().now().to_msg()
    state = VisionState()
    state.sequence, state.stamp, state.reset_counter = sequence, stamp, 0
    state.source_id = 'doc_synthetic_state_session_1'
    state.frame_id, state.body_frame_id = 'fc01_vision_ned', 'base_link_frd'
    state.initialized = state.tracking = state.calibration_valid = True
    state.valid_fields, state.quality = 71, 1.0
    state.position, state.velocity = [0.0, 0.0, -0.5], [0.0, 0.0, 0.0]
    state.attitude = [1.0, 0.0, 0.0, 0.0]  # explicit internal wxyz
    command = NavigationCommand()
    command.sequence, command.stamp, command.reset_counter = sequence, stamp, 0
    command.source_id, command.frame_id = 'doc_synthetic_plan_session_1', 'fc01_vision_ned'
    command.mode, command.valid = 1, True  # Hold; no position/yaw/sticks fields enabled
    state_pub.publish(state)
    command_pub.publish(command)

node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

若验证上面的 Odometry adapter，改用以下原始模拟源，并仅让 adapter 发布 canonical state。它每次生成同 stamp 的 Health/Odometry；所有“确认”只描述这个合成例子：

```bash
python3 - <<'PY'
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from jikl_fc01_system.msg import VisionHealth
rclpy.init()
node = Node('doc_synthetic_raw_source')
odom_pub = node.create_publisher(Odometry, '/fc01_demo/raw/odom',
    QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
health_pub = node.create_publisher(VisionHealth, '/fc01_demo/raw/health', 5)
sequence = 0

def publish():
    global sequence
    sequence += 1
    stamp = node.get_clock().now().to_msg()
    odom = Odometry()
    odom.header.stamp, odom.header.frame_id = stamp, 'demo_world_enu'
    odom.child_frame_id = 'demo_sensor_flu'
    odom.pose.pose.position.z = 0.5
    odom.pose.pose.orientation.w = math.sqrt(0.5)
    odom.pose.pose.orientation.z = math.sqrt(0.5)  # ROS xyzw fields, ENU yaw +pi/2
    health = VisionHealth()
    health.stamp, health.sequence, health.reset_counter = stamp, sequence, 0
    health.source_id = 'doc_synthetic_vio_session_1'
    health.frame_id, health.body_frame_id = 'demo_world_enu', 'demo_sensor_flu'
    for name in ('reset_valid', 'initialized', 'tracking', 'calibration_valid',
                 'frame_alignment_valid', 'time_sync_valid', 'extrinsics_valid',
                 'metric_scale_valid', 'position_valid', 'velocity_valid',
                 'attitude_valid', 'angular_velocity_valid', 'quality_valid'):
        setattr(health, name, True)
    health.angular_velocity_source, health.quality = 'doc_synthetic_gyro', 1.0
    health.extrinsics_id = 'DEMO_IDENTITY_MOUNT_NOT_CALIBRATED'
    health.alignment_id = 'DEMO_ENU_NED_NOT_CALIBRATED'
    health_pub.publish(health)
    odom_pub.publish(odom)

node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

可在同一独立演示环境查看 `ros2 topic echo /fc01_demo/vision/state --qos-reliability best_effort`。示例只证明合成接口格式/转换路径；真实竞赛相机仍需提供原始 topic、消息类型、sample 时间、跟踪与 reset 语义、公制尺度、外参和航向对齐证据。

本轮另有真实 ROS 合成 smoke 检查，独立手动运行，不纳入 unittest 自动发现：

```bash
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/local_setup.bash
source /tmp/fc01_vision_ros_20261031/install/share/jikl_fc01_system/local_setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=201 /usr/bin/python3 tests/ros_vision_smoke.py
```

预期 `REAL_ROS_VISION_ADAPTER_PASS valid_conversion=1 raw_conflict_revoked=1 fmu_publishers=0`。它只启动独立 adapter 与 `/fc01_demo/**` 合成源，检查 ENU 位置、child-frame 速度、有效位，以及两个原始 Odometry publisher 导致 mask=0 撤销和 adapter 没有 fmu 发布端点。本轮已得到此结果；未启动实际 Native 控制 Node，输入也不是真相机/飞控数据。


### 已有其他主仓库时

本地基线是从本目录源码新建的快照，不包含远端历史；不要用允许无关历史的强制合并来掩盖这一点。交付的 `dist/FC01_vision_navigation_20261031.patch` 只包含开发提交，可先在已有导航修复主仓库的新分支检查（把示例 your-navigation-repair-branch 替换为实际修复分支名）：

```bash
git switch -c feature/vision-navigation your-navigation-repair-branch
git apply --check /path/to/FC01_vision_navigation_20261031.patch
# 检查通过后应用，并按该仓库流程提交和审查。
git apply /path/to/FC01_vision_navigation_20261031.patch
python3 -m unittest discover -s tests -v
```

补丁检查失败时，对照本地基线 `6b440ab` 和现有修复，逐项保留 `rc_navigation.hpp`、权限仲裁、Core恢复/提交航向、`publication_guard.hpp`/`publication_boundary.hpp` 双sink及重采样检查、node订阅/时钟和配置映射。原推力/PID/保护 YAML、RC汇合、安装/boot文件没有被本轮替换。真机部署与服务切换应在源码合并及设备验收安排后单独执行。

`position_error` 是水平 XY 范数；高度目标与响应分别看 `target_height/height`。不能把水平误差直接当作三维定位误差。
