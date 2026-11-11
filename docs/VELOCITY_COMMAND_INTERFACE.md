# VelocityCommand 接口与独立只读诊断

最新工程状态见 [整体验证与修改](FC01_ENGINEERING_REVIEW_20261111.md) 和 [下一阶段准备](FC01_NEXT_STAGE_PREPARATION_20261111.md)。下文记录各阶段接口决策，早期提交和测试数量是历史记录；当前源码以最新验证清单为准。

后续五项边界修复及验证见 [视觉接口审查修复记录](VISION_INTERFACE_REVIEW_FIXES_20261111.md)。本页的诊断参数均为启动时配置，运行中修改会明确拒绝；以新参数重启独立诊断进程后生效。

完整文件核对、干净构建和当前 215 项 Python 补验见 [后续验证记录](VISION_INTERFACE_FOLLOWUP_20261111.md)，包含本轮保存的原始输出及日志归档说明。

想按执行顺序读代码，可以从 [视觉代码阅读导引](VISION_CODE_READING_GUIDE_20261111.md) 开始；其中说明实际状态、目标速度、缓存、时间和离线预览各自负责什么。

本次实现接收约定的速度消息、记录诊断、导出测试报告。诊断节点没有控制发布权，不把速度写入现有控制器，不启动起飞，不改变 RC、Offboard 或故障恢复状态机。正式导航服务保持 `visual_navigation_enabled=false`，也不会自动启动这个诊断节点。查看视觉消息不需要打开视觉飞行开关。

实现基于当前源码 `950ac04`；交接材料中的 `b388f26` 是该提交的祖先，之间已有四个后续修复提交。原起飞路径和以下参数继续保留：目标高度 `1.5 m`、`aa_hover_thrust=0.50`、`aa_takeoff_reserve_thrust=0.04`、`aa_thrust_max=0.80`。这些数值不是本次接口接入的调参结果。

## 两类输入的职责

| 通道 | 传递的数据 | 本次处理 |
| --- | --- | --- |
| `nav_msgs/msg/Odometry`，配合 `VisionHealth` | 飞机自身的实际位置、速度、姿态与估计健康信息 | 保留现有显式 Odometry adapter；真实尺度、外参、时间与航向对齐仍需视觉方确认 |
| `jikl_fc01_system/msg/VelocityCommand` | 上层希望飞机执行的速度 | 独立订阅和诊断，不进入控制闭环 |
| 现有 `NavigationCommand` | 既有 Hold/Cruise/Follow/Land 内部运动输入 | 本次不将 VelocityCommand 转换或转发到这里 |

Odometry 的速度是测量值，VelocityCommand 的速度是目标值。不能把目标速度当作当前速度反馈。现有 adapter 将明确的原始坐标、健康信息和标定转换为 `VisionState`；它没有把视觉信息注入 PX4 EKF。本次也不建立这种注入路径。

未来接入控制时，数据流应为“可信实际状态 + 经校验的目标速度 → FC01 外环 → 现有权限及最终发布检查 → PX4 姿态与角速度控制”。视觉节点不能绕过 FC01 发布最终姿态、推力或电机控制量。

## 消息合同

完整 ROS 类型固定为 **`jikl_fc01_system/msg/VelocityCommand`**，默认话题固定为 **`/fc01/vision/velocity_command`**。视觉端应使用这个完整类型；仅有相同的消息名称和字段、却位于另一个包，不视为已验证兼容。现有 `NavigationCommand` 不改名、不替代这个消息。

```text
uint16 schema_version 1
uint64 sequence
builtin_interfaces/Time stamp
uint32 reset_counter
string source_id
string frame_id
bool valid
float64 vx
float64 vy
float64 vz
```

字段与交接文件一致，没有添加协议字段。

| 字段 | 发送端必须明确的含义 |
| --- | --- |
| `schema_version` | 默认值为 `1`；其他版本不能当成本版本有效消息 |
| `sequence` | 非零、在一次发送会话内推进；重复、回退和重启分别记录，不以重复消息续期 |
| `stamp` | 指令计算时刻，采用双方约定的 ROS/system 时钟；不能用接收时间冒充 |
| `reset_counter` | 发送方重置计数；本阶段只记录变化，没有绑定 `VisionState.reset_counter` |
| `source_id` | 非空来源标识，建议包含发送会话身份；来源变化不自动证明控制权转移 |
| `frame_id` | 必须显式填 `body_frd`；消息默认字符串为空，注释不会产生默认值 |
| `valid` | 本帧是否有效；false 不代表授权延续上一帧非零运动 |
| `vx/vy/vz` | 目标速度，单位 m/s，必须为有限数值 |

`body_frd` 是三维机体系：X 前、Y 右、Z 下。`vx>0` 向前，`vy>0` 向右，`vz<0` 向上。任意姿态下，将完整三维机体速度转换到世界系需要明确对应姿态与时刻；本次诊断只记录消息，不做控制坐标转换，也不将其强行映射为旧 Follow 的四路归一化 sticks。

消息没有 yaw 或 yaw rate 字段。本次不据此生成航向请求，现有航向行为保持原语义。零速度目标也不自动等于位置锁定；真正保持位置还需要可信位置反馈和明确的控制模式。

现有文件中的“忽略无效帧或保持上一状态”注释只可用于理解接收缓存，不应理解为无限维持上一帧非零运动。诊断可以保留最近原始数据用于查错，但没有任何运动执行权；未来控制接入前必须定义无效、过期和失联的退出策略。

## 启动只读诊断

先在对应平台构建并 source 本包安装环境，确认类型存在：

```bash
source /opt/ros/humble/setup.bash
# 另行 source 本次构建的 jikl_fc01_system 安装环境。
ros2 interface show jikl_fc01_system/msg/VelocityCommand
```

独立接口测试可在参与测试的每个终端设置相同的隔离环境；这不会启动生产控制器：

```bash
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
```

接收真实视觉端消息时，应使用双方实际约定的 Domain、网络范围和 QoS，而不是沿用隔离演示值。诊断节点启动命令：

```bash
ros2 run jikl_fc01_system velocity_command_diagnostic.py --ros-args \
  -p topic:=/fc01/vision/velocity_command \
  -p qos_depth:=5 \
  -p reliable:=false \
  -p output_dir:="$HOME/FC01_navigation"
```

`reliable=false` 为 best effort，默认 volatile。若双方约定使用 reliable，可显式设置 `-p reliable:=true`，并确认发送端 QoS 兼容。脚本独立运行；停止此诊断进程不会向飞控发送停止、解锁或模式切换命令。

诊断参数与飞行参数分开：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `topic` | `/fc01/vision/velocity_command` | 订阅话题 |
| `qos_depth` | `5` | 有限接收深度 |
| `reliable` | `false` | 接收 QoS 可靠性 |
| `expected_source_id` | 空字符串 | 未绑定发送源；配置后只匹配该会话来源，单一 publisher 数量不能替代来源绑定 |
| `output_dir` | 空字符串 | 默认不写文件；上述命令显式选择报告目录 |
| `snapshot_period_sec` | `1.0` | 诊断快照节奏，不是指令 TTL |
| `log_min_interval_sec` | `0.2` | 普通诊断日志最小间隔 |
| `log_max_bytes` | `262144` | 单份诊断日志轮转上限 |
| `log_backup_count` | `1` | 保留一份轮转日志 |
| `timeout_sec` | `0` | 未配置指令接收超时诊断阈值 |
| `max_source_age_sec` | `0` | 未配置源时间年龄诊断阈值 |
| `future_tolerance_sec` | `0` | 未配置未来时间诊断容差 |
| `max_speed_mps` | `0` | 未配置速度诊断上限 |

四个默认 `0` 的阈值表示 **未配置/未知**，不是无限制飞行许可，也不是已约定的控制限幅。消息可以完成格式检查，但不能由此宣称满足飞行接入条件。需要测试这些诊断时，可显式传入双方选定的试验值；诊断输出应保留配置状态，避免把开发阈值冒充正式协议。无论如何配置，诊断节点的 `control_authority` 始终为 false。

需要验证来源绑定时，另外传入 `-p expected_source_id:=<双方约定的发送会话标识>`。未绑定源或四个数值阈值有任意一个未配置时，`cache_usable=false`；这不影响观察原始输入与问题记录。

## 查看数据和导出报告

使用上述 `output_dir` 后，诊断产生：

```text
$HOME/FC01_navigation/velocity_command_diagnostic.json
$HOME/FC01_navigation/velocity_command_diagnostic.log
$HOME/FC01_navigation/velocity_command_diagnostic.log.1
```

查看最近快照与日志：

```bash
python3 -m json.tool "$HOME/FC01_navigation/velocity_command_diagnostic.json"
tail -n 20 "$HOME/FC01_navigation/velocity_command_diagnostic.log"
```

应联合检查原始 stamp 与本机接收时间、来源、frame、sequence、reset、valid、原始速度、诊断原因、计数和阈值是否已配置。保留的历史样本用于解释问题，不能当成当前有效运动需求。节点停止后文件仍可能存在，应查看记录时间和进程状态；文件存在不等于诊断仍在运行。

`accepted` 计数仅表示样本通过当时已启用的检查。`freshness=CURRENT` 需要配置接收和源时间策略并同时满足两种年龄检查；`receive_freshness` 单独描述接收年龄，不能代替源测量年龄。还应查看 `cache_usable`、`cache_errors`、`current_candidate`、`expected_source_id`、发布者数量与阈值配置。非法消息、源丢失/冲突、过期与时钟回退会撤销候选；`historical_last_accepted_message` 是历史诊断，不是当前目标。新鲜有效输入可以恢复诊断缓存，但不能抹掉 `last_anomaly`。任何字段都不能单独表示允许运动；`qualification=DIAGNOSTIC_ONLY`、`cache_scope=DIAGNOSTIC_ONLY` 与 `control_authority=false` 始终保留。

来源/reset 的可信历史最多 32 项。满额后新会话明确报 `STREAM_HISTORY_CAPACITY_REACHED`，不淘汰旧水位使重放重新有效；原有会话的新鲜正常数据仍可恢复。需要开启新的独立诊断会话时可重启诊断进程，这不会清除飞机的故障或改变任何飞行权限。非法消息不占可信历史容量。

同一来源切换 reset 时可以重启序号，测量 stamp 仍须晚于该来源已接受的全部周期；reset 不重置双方约定的 ROS/system 时钟。旧或相同时间明确报 `SOURCE_STAMP_NOT_ADVANCING_ACROSS_RESET`。合法撤销同样保留时间水位，被拒绝的消息不推进水位。

本版本 `navigation.sh report` 会只读收集这三个固定文件，并保留原报告大小限制、日志截断标记和文件摘要。调用当前版本入口：

```bash
bash /path/to/current-release/navigation.sh report
```

将路径替换为实际使用本版本源码/安装包的目录。旧安装包中的报告程序不会自动获得这个新增收集能力。若把 `output_dir` 设为其他目录，默认导航报告不会扫描该目录。当前正式服务不自动启动或监督诊断节点，也不把它的消息授权给飞控。

## 离线验收与后续边界

离线验证包含消息生成、字段和时间/序号诊断、隔离 ROS 收发、QoS/断流/多源/重启、日志轮转与报告收录。真实 ROS 冒烟检查应确认诊断进程没有任何 `/fmu/in/**` publisher，未启动控制服务或 Agent。测试数据必须标记为合成数据。

在仓库根目录、已 source 新构建安装环境的终端中，可运行真实 ROS 冒烟检查：

```bash
ROS_DOMAIN_ID=188 PYTHONDONTWRITEBYTECODE=1 \
  python3 tests/ros_velocity_command_smoke.py \
  --report /tmp/fc01_velocity_ros_smoke.json
```

`188` 只是隔离示例，应先选空闲的非零 Domain。脚本使用 localhost 发现、清理外部 DDS 发现配置，若发现 `/fmu/**` 或生产 `fc01_system` 节点会拒绝继续。它只运行合成发布者与独立诊断；其中 `1 s` 超时及更快快照/日志节奏均为测试 fixture，不写入正式配置。`--diagnostic` 可选择诊断脚本路径，默认使用仓库源码。

本次不把 VelocityCommand 接入 `Runtime`、不启用视觉飞行、不改原推力/高度参数。之后还需要双方明确真实数据的时钟对应与采样偏差、传感器外参及航向对齐、TTL 和速度限制、状态/命令 reset 的含义、来源与会话切换规则、无效与失联退出，以及与现有 NavigationCommand 的互斥关系。下面的纯计算模块已经提供适配与预览工具，但没有取得真实控制权。

## 视觉状态的纯计算校验层

`native/tools/vision_state_validation.py` 不创建 ROS 节点。它复用 `vision_odometry_adapter.py` 的实际转换公式；现有 Odometry adapter 的运行路径、飞行控制核心与协议字段均不变。可从源码工具目录或安装后的脚本目录导入本模块。

| API | 用途 |
| --- | --- |
| `AdapterConfig` | 复用原显式坐标、外参、来源和标定配置；没有自动猜测相机 optical 坐标的路径 |
| `ClockPolicy(offset_ns=None, confirmed=False)` | 明确 `local ROS time = source stamp + offset_ns`；只有确认后才应用，显式零表示已确认同一时钟 |
| `ValidationConfig(adapter, ...)` | 配置年龄、质量、连续性和原始协方差上限；阈值默认 `None`，表示未知 |
| `SampleMetadata(...)` | 本地声明 pose/twist covariance 是否有效，以及明确的 `relocalization_id`；不增加 ROS 消息字段 |
| `Validator.receive(odometry, health, received_at, ros_now_ns, metadata=None)` | 处理一个已精确配对的样本；返回 `ValidationResult` |
| `Validator.current(now, ros_now_ns)` | 重新检查缓存当前年龄和时钟；失败后撤销当前缓存 |
| `Validator.drain_events()` | 取出最多 128 条待处理事件；累计计数和最后异常保留 |

`received_at` 和 `now` 由本机 monotonic 时钟提供，`ros_now_ns` 是同一次观测中的本机 ROS/system 时间。原始 Odometry 和 Health 的测量 stamp 必须相同。确认的 offset 只应用到转换用副本，原始消息不修改；没有时间估计器或“拿到达时间当测量时间”的后备路径。

`receive_timeout_sec`、`max_source_age_sec` 和 `future_tolerance_sec` 未全部配置，或时钟未确认时，不建立当前有效状态缓存。前三项中前两项须为正数，未来容差可以显式为零。`min_quality`、`nominal_quality_min`、三个连续性上限与两个协方差上限未配置时，会列在 `diagnostics.unknown`；通过数据适配不表示这些未知项已经得到验证。结果始终带 `control_authority=false`、`qualification=DIAGNOSTIC_ONLY`。

新接收样本在替换缓存前也检查旧缓存是否已过期，因此不依赖定时查询恰好先执行；超时/恢复事件及最后异常不会被随后正常样本覆盖。接收和查询共同维护独立时钟水位，包括没有当前缓存的阶段，防止查询后的时间回退绕过检查。

质量检查使用现有 Health 的 tracking bool 和显式 quality：tracking=false 为 LOST；有 nominal 阈值时区分 NORMAL/DEGRADED；未配置阈值时为 TRACKING_UNCLASSIFIED。低于显式 `min_quality` 时拒绝建立缓存。跟踪恢复且新的序号、时间与数据检查通过后可重新接收，不会永久锁死诊断层。

source/frame、估计器 reset 或显式 relocalization 变化分别保留 epoch 事件。未知来源或坐标仍需匹配 `AdapterConfig`，不会因事件发生就自动信任。`relocalization_id` 应在同一地图周期内保持一致。显式新 epoch 可以重新建立连续性基线；没有 epoch 证据的跳变只在配置了对应位置/速度/姿态阈值时拒绝，四元数 q 与 -q 视为同一姿态。拒绝与缓存过期不会删除最后可信连续性基线，因此不能用“A→异常 B→重复 B”把异常静默洗成新基线；恢复到可信连续轨迹或明确新 epoch 后可以重新建立缓存。

可信 epoch 历史只在样本实际通过检查时写入。退役的 reset 或 relocalization 周期再次出现，即使时间仍在 TTL 内、序号重新推进，也会以 `RETIRED_EPOCH_REPLAY` 拒绝，不能回放旧坐标跳过连续性检查。未通过的 epoch 不占用历史容量、不使当前可信 epoch 退役。`epoch_history_capacity` 默认 128，范围 1–4096，是资源上限；历史满时新 epoch 被明确拒绝为 `EPOCH_HISTORY_CAPACITY_REACHED`，不会静默淘汰旧记录而让回放变成“新周期”。当前可信周期仍可继续更新。若离线测试确实需要更多不同周期，应显式创建新的 `Validator` 或为新会话选择适当容量；这只重建离线诊断对象，不清理飞机故障或授权真实控制。

reset/relocalization 可以重新建立位姿与序号基线，不能使已经确认的同一测量时钟倒退。所有新 epoch 的测量 stamp 仍必须严格晚于最后通过检查的样本；相同或更旧 stamp 以 `MEASUREMENT_CLOCK_NOT_ADVANCING` 拒绝，且不使当前 epoch 退役。全局测量时间水位只由实际接受的样本推进。源时钟确实重新启动时，需要明确新的时钟对应与离线 `Validator` 会话，不能仅用 pose reset 绕过时钟检查。

协方差只有显式声明后才检查原始 6×6 矩阵的有限性、非负对角线、对称性、半正定与可选六项方差上限。PSD 检查使用按各轴方差归一化的纯计算实现，不依赖 NumPy；数值舍入容差不是飞行阈值。未声明 covariance 为 UNKNOWN；显式全零单独标为 DECLARED_ZERO，不自动推断高可信。检查的是原始 Odometry 对应坐标系下的方差，不是已完成外参和杆臂 Jacobian 的协方差。输出 `ConvertedState` 的两个 covariance valid 位继续为 false，数组保持未知。

调用示意如下，`adapter_cfg` 应由实际明确的接口/标定参数创建，不能用演示单位外参冒充实际标定：

```python
from vision_state_validation import ClockPolicy, ValidationConfig, Validator, SampleMetadata

# adapter_cfg: 已有 AdapterConfig；默认策略只允许诊断未知项。
validator = Validator(ValidationConfig(adapter=adapter_cfg))
result = validator.receive(raw_odometry, matched_health, local_monotonic_s,
                           local_ros_ns, SampleMetadata())
assert result.control_authority is False
print(result.diagnostics)  # 未确认时钟/阈值时 accepted 为 false。
```

离线单测：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests \
  -p test_vision_state_validation.py -v
```

## 目标速度到控制外环的离线预览

`native/src/jikl_fc01_system/include/jikl_fc01_system/velocity_command_input.hpp` 提供纯 C++ `fc01::velocity_input::Session`。它没有被生产 `Runtime` 或 `fc01_system_node` 包含，不创建 PX4 消息或权限租约。

实际 `vision::State` 与 `velocity_input::Target` 分开输入；`from_message()` 保留 VelocityCommand 原字段。`Mode::Disabled` 不产生有效预览，`Mode::VelocityPreview` 只计算：

```text
完整三维机体 FRD 目标速度
          ↓ 使用对应实际四元数 R(q)
世界 NED 目标速度 → 现有候选速度限幅
          ↓ 减去实际世界速度
速度误差 → 现有候选速度增益 → 加速度/倾角限幅 → 姿态预览
```

不能用仅 yaw 的旋转替代完整三维变换。指令没有 yaw 字段，预览使用调用方提供的既有可信航向参考。`world_vertical_velocity_preview` 只展示世界垂向速度目标，没有送入原高度控制器。状态估计器 reset 与命令生产者 reset 分别记录，不要求两个计数相同。

`Policy` 默认值不会允许预览。需要显式给出 state/command TTL、最大测量延迟、命令速度上限、姿态与指令采样偏差上限、姿态连续性上限；未来时间容差可为零。来源/时钟/数据异常锁存在本次离线 Session，不会被正常尾帧消除；`begin_attempt()` 只清理离线会话，没有清理真实飞机故障或取得控制权的能力。预览输出始终 `control_authority=false`。

预览接口要求上游先完成状态适配与校验。其可选 covariance 检查复用现有 C++ helper，仅检查有限性及对角线，不能替代完整 PSD 检查；本次原始协方差 PSD 校验由上面的 Python 状态模块负责。没有自动把 Python 校验结果送入生产控制器或把原始 covariance 宣称为已转换。

运行预览模块回归，不建立 ROS 图或硬件端点：

```bash
FC01_PREVIEW_BUILD=$(mktemp -d /tmp/fc01_velocity_preview.XXXXXX)
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include \
  native/tests/test_velocity_command_input.cpp \
  -o "$FC01_PREVIEW_BUILD/test_velocity_command_input"
"$FC01_PREVIEW_BUILD/test_velocity_command_input"
```
