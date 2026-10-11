# FC01 M1 / Agent B: 0.4.4-offboard-flight-dev

> **DEV CANDIDATE / NOT CLEARED FOR FLIGHT — 离线开发件，禁止视为真机放飞许可。**
> 日期：2026-10-09。用户授权仅限独立 DEV 源码、离线测试及构建；未连接 FC01，未 Arm、Takeoff、刷机、改参数或驱动电机。

## 1. 输入、来源和限制

- **唯一开发基线**：用户上传的 `FC01_M1_B_DEV_INPUT_20261009.tar.gz`，解包根目录 `0.4.4-offboard-flight-dev/`。原始文件与 DEV 分开保存，没有原地修改输入。
- 审查契约：用户提供的 `FC01_M1_AGENT_B_APPROVED_PROMPT_20261009.txt`、`FC01_M1_AGENT_B_DEVELOPMENT_CONTRACT_20261009.txt` 和智能体 A 完整 `FC01_M1_A_ARCHITECTURE_REVIEW_20261009.md`。
- 上传工程顶层 `VERSION` 曾仍是 `0.4.3-rc-interface`，本 DEV 改为 `0.4.4-offboard-flight-dev`；其历史文档/旧日志不是本候选件实际飞行验收。
- `workspace/src/px4_msgs` 原封保留；契约 pin `392e831c1f659429ca83902e66820d7094591410`，**原始源码包未含可核对提交的 `.git` 元数据**，因此可证实逐字节不变，但无法仅从压缩包独立认证该 Git commit。
- 原文件 `BASELINE_COLCON_BUILD.log` 是用户环境下基线两包构建记录，**非本次候选源码**的构建记录。

## 2. 范围 / 实现

源框架、话题/RC 适配、ACK matcher / transaction tracker、authority state machine、stage-one exact-zero、最终 ROS target/mode publisher、Ownership 检查保持原架构，直接在其上增量变更：

| 代码 | 本次变化 |
| --- | --- |
| `attitude_altitude_core.hpp` | ArmedGround 的 armed-observation dwell；不把 armed 后 `at_rest=false` 当无条件故障；ThrustRamp 要求新 landed timestamp、`landed=false`、正高度及上升速度联合驻留，禁止时间假离地；地面推进失败超时闭锁；Ascend/HeightHold/Descent/Landed 继续复用原 PI+slew；触地防抖、零推力确认和严格地面 Disarm；新增 `SafeDisarmed -> ReleaseOffboard -> Idle` mode ACK/status 交接；分类故障；新增相对高度下界和 Z/VZ 跳变诊断。 |
| `vertical_controller.hpp` | 高度低于 z0 的有界负偏差检查；M1 配置的 `height_lower_margin_m` 同步传递给现有 controller，避免内外边界不一致。 |
| `publication_guard.hpp` | 非零推力只接受已捕获 yaw、水平目标 quaternion（2° 角容差）；真实测量倾角、最大高度和低于地面的高度均 gate；独立 `disarm_publication_policy` 需本次 Arm ACK、地面新 landed/status、阶段与 Z/VZ 再确认；关闭飞行门控时只允许 disarmed-ground zero-thrust 的 ReleaseOffboard 退场。 |
| `publication_boundary.hpp` | 复用原 sink 风格，为 Disarm 和 ground-only Release 增加最后一刻审核包装。 |
| `runtime.hpp` | 接入新状态机命令；`176` 离开 Offboard 时使用原 transaction tracker 的 watermark / target / status 配对；原 Arm/Disarm `400` 仍严格顺序执行；把最新观测与 release/shutdown 上下文交给最终边界。 |
| `fc01_system_node.cpp` | 将常规 Companion `400 param1=0` 绑定最终 Disarm sink gate；将地面释放 `176 param1=1 param2=7` 绑定地面释放 gate；引出故障类别；保持 RC/Kill 原物理语义不变。 |
| `attitude_ros_parameters.hpp`、`config/attitude_altitude.yaml` | M1 观察参数映射及数值验证；四个 flight gates 全部 `false`，悬停推力、控制增益、气动数据均明确未标定。 |
| `tests/test_m1_safety.cpp` + 既有测试调整 | 新增最终 Disarm sink TOCTOU、roll/yaw 注入、release 零推力、未离地、ArmedGround dwell、pending Offboard + STOP + sequential 176 ACK/status、时间与估计跳变负例。保留所有原有测试文件。 |

## 3. 基线安全约束（不得在没有真实校准证据时打开）

`config/attitude_altitude.yaml` 中的 `flight_actions_enabled`, `arm_enabled`, `takeoff_enabled`, `flight_parameters_confirmed` 均为 **false**。其余 `aa_hover_thrust`, PID、slew、阈值仅为**未校准候选值**，不能依据离线通过结果将它们确认。`rc_takeover_confirmed` 是单独的历史配置字段，不会替代四门控，`autonomous_failsafe_confirmed=false`。

设计上仅控制 **水平姿态（保持启动 yaw）和相对高度**；不具备 XY 定点/定位、导航、SLAM、避障或实机保证的自动安全着陆。

## 4. 离线验收与构建边界

- **CONFIRMED**：修改后运行 `bash scripts/test.sh`，**14 个 C++ 测试可执行文件**全部执行 PASS（从原 13 个增加 1 个）、ROS adapter 对**模拟 rclcpp/px4_msgs fixture**语法编译检查 PASS、**18 个 Python unittest / OK**。全部原用例保留，代码以 `g++ -std=c++17 -Wall -Wextra -Werror -pedantic` 编译。
- **CONFIRMED**：同一 90° 横滚注入，用户原件 `ALLOWED`、本 DEV `DENIED`；详见 `RED_GREEN_Q_ENVELOPE.log`，复现 C++ 源在交付的证据包。
- **UNKNOWN**：本次环境为 Debian 13，无 `/opt/ros/humble` 与 `colcon` 命令；**未运行候选件真实 ROS 2 Humble `colcon build` / link / DDS**。不能用 fixture 代替。
- **UNKNOWN**：Ubuntu 22.04 x86_64 上对**本候选**的原生完整构建与设备采集。
- **UNKNOWN**：ARM64 香橙派对同一源码 native colcon 构建、Agent/USB/serial/DDS 性能。
- **UNKNOWN**：真实 FC01 PX4 v1.16 厂商固件的 `STABILIZED` 可选择性、状态/ACK 时间一致性、失联后的无 XY 模式 fail-safe、推力模型、接地检测、任务安全。

## 5. BLOCKER/HIGH 负例复核矩阵

这里的 PASS 专指本候选**离线 C++ 测试**；任何 PX4 固件或真机验收均单列 UNKNOWN。

| 智能体 A 风险 | 本次代码对应与测试 | 离线 | 真实系统 |
| --- | --- | --- | --- |
| BLOCKER-01 ArmedGround 无有效观察/时间假离地 | Armed dwell + landed 后续时间戳 + Z/VZ + liftoff dwell + timeout；`test_m1_safety` | PASS | UNKNOWN |
| BLOCKER-02 退出/故障不分地面空中 | 正常 Descent/RequestDisarm、GroundAbort/ControlledDescent/Px4Handoff/UnknownAirborne 分类，未知空中绝不普通 Disarm；不过真失联完全受 PX4 自身策略支配 | PARTIAL | UNKNOWN |
| BLOCKER-03 Disarm sink TOCTOU | 最终 Command 400 param1=0 前实时重采样，缺 ground/contact/fresh 时 sink 调用 0；`test_m1_safety` | PASS | UNKNOWN |
| BLOCKER-04 真失联 PX4 模式可用性 | 明确未验证，不宣称自动降落 | UNKNOWN | UNKNOWN |
| BLOCKER-05 电机/ESC/电池推力实测 | 所有飞行门控默认 false，参数未确认 | FAIL-CLOSED | UNKNOWN |
| HIGH-01 target quaternion 容许 90° roll | 原件允许、DEV 拒绝；相同输入原/新 probe；yaw 方向注入拒绝 | PASS | UNKNOWN |
| HIGH-02 touchdown 过早认为 landed | 新 land timestamp、接触驻留、零推力 ramp、触地反弹重返 Descent、继续 grounded 证据 | PASS | UNKNOWN |
| HIGH-03 地面 Stop/退出 Offboard race | pending `176` + STOP -> 等 ACK+新状态 -> disarmed-ground Release -> 新 ACK+稳定模式状态才 Idle | PASS | UNKNOWN |
| HIGH-04 高度负下界及 z/vz 跳变 | 双层 envelope + estimator reset 严格 fail-closed；连续物理真实性尚需测量 | PASS (边界) | UNKNOWN |
| HIGH-05 真实 Disarm 阶段竞争 | 最终 Disarm source + sink 双闸检查、严格地面重采样 | PASS | UNKNOWN |
| HIGH-06 400/176 ACK 时序 | 保留旧 `Runtime::ack` 原语；先 Arm(400)、后 Disarm(400) 无并发，旧、伪和 late ACK 用例 | PASS (合成) | UNKNOWN |

## 6. 已知风险，禁止误判

1. 逻辑状态机能正确做离线模拟，**不等于**现实高度闭环受控：参数、悬停推力、接地检测、姿态轴方向、实际时间戳与滤波器品质均未验证。
2. 任何意外节点崩溃、Companion 断电、DDS Agent 中断或 RC Kill，Companion 无法保证提供继续悬停或受控下降；PX4 的真实 `COM_OF_LOSS_T` 与 NAV fail-safe 仅可在独立安全许可后读取核实，**本次未写参数**。
3. `VehicleCommand` 手动模式释放使用 `176 (param1=1,param2=7)`；能否切换、`NAVIGATION_STATE_STAB` 是否可选、ACK 与 status 变化需要厂商 FC01 验证。模式发布可能在失联前失败；对这种失败系统闭锁，不宣称恢复成功。
4. 软件在失效时拒绝普通 Disarm，但**不能替代独立的 RC Kill / PX4 failsafe**；Kill 的副作用可能是立即停机。
5. 长时间地面释放等待需 ROS/DDS 有效；实机模式切换是否及时无证据。若 `can_set_nav_states_mask` 禁止 STABILIZED 则不会直接强制释放。
6. 位姿估计跳变判据只是异常检测，不是状态估计真实性证明；高度/速度只有 Z/VZ，**不存在水平漂移约束**。
7. `px4_msgs` Git 源提交不可由无 `.git` 的随包目录自行证明；只能通过文件哈希比较开发前后不变。厂商固件 ABI 仍 UNKNOWN。

## 7. 下一阶段（交智能体 C 的独立源码审查，当前不能飞）

1. 从本源码 ZIP 重建并独立比对所附统一 diff、文件 SHA256；检查 `src/jikl_fc01_system/include`, `fc01_system_node.cpp` 的真实 sink 和 `Command` 映射，尤其 ReleaseOffboard 零推力心跳和 node 的 publisher double-check。
2. 在有 ROS Humble 的 **Ubuntu 22.04 x86_64** 以固定 `px4_msgs` 消息，真实 `colcon build --packages-select px4_msgs jikl_fc01_system`（或按仓库实际依赖顺序构建），保存完整 stdout/stderr。要证明真实 API/callback 链接，不可仅用 fixture。
3. 在同一份源码的 **ARM64 香橙派** 原生编译同两包；不能复用 x86_64 的 ELF，需核实 USB/serial、Agent 库架构、权限和消息 ABI。
4. 对 PX4 厂商固件失联行为、RC Kill、常规 Disarm、模式切换、Z/VZ 与 landed 的真实性、动力系统进行独立安全计划评审；不得从当前 DEV 包直接有桨起飞。

## 8. 交付追溯

外置交付证据包含原始 `scripts/test.sh` 终端输出、90° 注入原/新对照源和日志、源/补丁/说明 SHA256、`colcon` 候选构建 UNKNOWN 状态。`README_zh.md` 保留原项目其他历史信息，本文件为当前 DEV M1 变更的优先说明。
