# 智能体 F — 0.4.0-attitude-altitude 完整源码交付

本次实际实现、新增测试并生成独立 source release；第一阶段禁止自动 Arm/Takeoff，第二阶段默认锁死。没有执行真机动作、安装服务或修改飞控参数。

材料冲突已先列在 `CONFLICTS.md`：依据 00 的真机事实→实际 r2 源码→v1.16 接口→手册→C 架构→推论顺序判断事实，按用户明确指令和 01/03/04 约束本次行为。主要裁决为固定捕获局部 yaw、stage-one armed 拒绝、继承后续实机验证的 ACK 规则、拒用 main 接口，以及 install 首次也不切 current。未得到证据的项目写 UNKNOWN。

## 1. 读到的基线与关键事实

已解压整个正式 ZIP，并阅读 00/01、02–05、独立任务书、实际 0.3.1 源码/脚本/配置/测试、A/B/C 三份原始报告和相关证据。FC01 手册已读；Orange Pi 手册审查了 ARM64/Jammy/Humble、USB、SSH、编译器等迁移相关章节。交接包 SHA256 检查通过；r2 的 ZIP/TAR 内 30 个常规源文件逐字节相同。

- 真机已验证 release：`0.3.1-ackfix-20261004T090116Z-Pdwbao`；本次从 r2 解出源码独立修改。
- px4_msgs commit 固定 `392e831c1f659429ca83902e66820d7094591410`；没有切换 PX4 main。
- 当前真机 XY/VXY/Heading 无效，Z/VZ 有效；新 profile 不以导航估计有效作为准入条件。
- VehicleStatus v1 使用 `/fmu/out/vehicle_status_v1`；新增两个 Attitude 消息 MESSAGE_VERSION=0；保留实际基线版本断言，没有给 Mode 虚构 MESSAGE_VERSION。
- verified ACK.from_external=false 不能被拒绝；原 command/target/pending/timestamp watermark/IN_PROGRESS/timeout 规则保留。Runtime::ack()/track() 和整个 ground core 原文未改。

## 2. 架构修改摘要

保留 ground MODE 和原 flight/sim 导航控制；新增互斥 `attitude_altitude`。ground 不创建飞行 setpoint publisher；flight/sim 创建 Mode+Trajectory；新 profile 创建 Mode+AttitudeSetpoint，禁止竞争 ownership。

纯 C++ 分离 attitude reference、vertical controller、状态机和 ownership；ROS adapter 负责 v1.16 消息、单调来源 stamp、实际 graph 和配对输出。局部 yaw 由 Hamilton quaternion 捕获并保持，roll/pitch target 为 level。

第一阶段持续零推力 prepare；连续健康 prestream 1.2 s 后发 Offboard，合法 accepted ACK 加较新实际 Offboard/disarmed status 才 READY。第二阶段是向上为正的级联高度/速度 PI + hover feed-forward，经 clamp/slew/anti-windup 输出 `[0,0,-Tcmd]`。实际离地后使用该控制器下降，确认落地并降推力到零后才顺序 Disarm，不借 NAV_LAND 替代新路径。

healthy setpoint 与 heartbeat 在同一受监督主循环发送；严重故障会锁存 SAFETY 并停止两者，没有独立假 heartbeat 线程。

## 3. 文件级 diff 摘要

| 文件或相对路径组 | 最终改动 |
| --- | --- |
| VERSION；src/jikl_fc01_system/package.xml | release 0.4.0-attitude-altitude；package 0.4.0 |
| include/jikl_fc01_system/attitude_reference.hpp | finite/norm 验证、局部 yaw 捕获、level quaternion、reset policy |
| 同目录 vertical_controller.hpp | NED 符号转换、cascade PI、显式参数 validation、clamp/slew/anti-windup、landing ramp |
| 同目录 attitude_altitude_core.hpp | 两阶段状态机、所有 state/health gates、ground reference、取消/下降/Disarm、关键信号故障 |
| 同目录 setpoint_ownership.hpp；attitude_wire.hpp；attitude_ros_parameters.hpp | profile publisher policy、真实字段编码、YAML 参数声明 |
| 同目录 runtime.hpp | 新 profile/operator routing、既有 pending 结果消费、顺序 400 watermark、shutdown latch；ACK/track 原文保留 |
| src/jikl_fc01_system/src/fc01_system_node.cpp | 新消息/版本断言、publisher 按 profile 创建、freshness/reset/ownership、配对发布、shutdown Arm gate、JSON/CSV |
| config/attitude_altitude.yaml | 所有新 gates=false；飞行参数 -1 sentinel；20 Hz 准备配置 |
| tools/fcctl.py；tools/pc.py | attitude-ready 完整等待、真实 TTY marker、单次危险动作拒绝、SSH 交互 TTY |
| tools/deploy.py；scripts/export_portable.sh | export 名称读取 VERSION；排除 git/cache/build/install/log/binary |
| scripts/install.sh | 独立 release、固定消息源校验、安装从不自动切 current |
| scripts/run.sh；scripts/test.sh | 新 profile、日志目录配置；保持 C++17 strict tests、无 pycache/core 产物 |
| tests/test_attitude_vertical.cpp；test_attitude_runtime.cpp；test_attitude_wire.cpp；test_socket.cpp；test_tools.py | 新数值、状态、协议、ownership、安全回归和脚本验收 |
| README_zh.md；docs/ | 完整安装/回退/smoke、原合同、冲突裁决、验收矩阵、审查/执行证据、迁移审查 |

表中 header 路径位于 `src/jikl_fc01_system/include/`；完整逐行差异在附带 unified patch。旧 velocity/ground/socket/protocol 核心、ACK 原测试、Agent/update/service 脚本和旧 profile YAML 均保持字节一致。

## 4. 安全门设计

- config：flight_actions_enabled、arm_enabled、takeoff_enabled 全 false；flight_parameters_confirmed=false；启用 master 但缺任何有效显式飞行参数时拒绝 startup。
- state：只有新 ACK+status 确认的 OFFBOARD_READY、disarmed/landed/at_rest、可信 z0、battery/RC/preflight 和 live interactive PING 才接受 TAKEOFF；Arm 发布口复查配置、连接和 shutdown latch。
- health：fresh finite attitude/Z/VZ、正常 reset/time、无 failsafe/HIL/calibration/conflict、20–50 Hz 且无 stall；失效停止 heartbeat/output。stage-one armed=true 立即显式 SAFETY。
- transaction：原 ACK matcher/track 不变；Arm/Disarm 的 400 不并发，继承 ACK watermark，超时/late arm 不恢复，不自动重试。
- cancellation：确认仍在地面时，LAND/断线/关闭直接减推力到零，不增加 hover thrust；不依赖 ASCEND 等阶段标签。已离地且 healthy 才执行受控下降；未决 Arm 或严重故障交还 PX4/operator。
- lifecycle：启动/连接/只读/prepare 不会 Arm；交互 PING 超时停止第一阶段；one-shot ready 等到实际结果并在断线时释放；shutdown 后禁止所有新控制任务。

旧 flight/sim 的 flight_enabled 与导航估计门按合同保留；本次第一阶段只使用新 profile，禁止选择 sim 绕过新三门。INTERACTIVE marker 是同用户可信 operator 会话联锁，不声称对恶意同用户程序提供认证。

## 5. 测试结果

`bash scripts/test.sh` 最终 exit=0：原 ACK lifecycle PASS；attitude/vertical/ownership PASS；runtime/state/ACK+status/顺序400/故障 PASS；wire/publisher plan PASS；socket 文件保护和 stamp regression PASS。Python 8 个测试为 7 PASS、1 native AF_UNIX SKIP；C++ native socket bind/listen 同样 SKIP，原因是执行 sandbox 拒绝。

所有 shell/Python 语法 PASS；整份 ROS adapter 用独立类型/API fixture 的严格 C++ 语法检查 PASS，**不是真实 ROS 构建**。安装/current/update/rollback 是真实文件系统 fixture，ROS/外部系统调用以 fixture 替代。基线不变项逐字节检查 PASS。独立审查在 f10801c 批准 source delivery，已关闭全部发现；审查不替代本次实际测试或硬件验收。

详细合同逐项矩阵、原始输出与 hashes 分别在 ACCEPTANCE_RESULTS.md、TEST_OUTPUT.txt、BASELINE_INVARIANTS.json；归档和 patch replay 另有外部 PACKAGE_VERIFICATION.txt。

## 6. 未验证事项：UNKNOWN

| 项目 | 原因/后续证据 |
| --- | --- |
| 0.4 完整 Ubuntu22.04/Humble build/link | 当前 host 是 Ubuntu24.04，无 Humble/colcon；实际 installer 被平台检查阻止 |
| 真实 DDS graph/QoS、0.4 Attitude Offboard 接受和 READY | 未连接飞控；需按第一阶段拆桨 smoke 执行 |
| 本执行环境 native AF_UNIX console | sandbox 拒绝 bind/connect；不能以 PTY fixture 声称 native PASS |
| Orange Pi A733 原生 build、时序、USB/串口/Agent 运行 | 只完成源码与相关手册审查 |
| hover/gains/limits、垂直动态/稳定性、真实 LandDetected 与 Disarm | 没有给定实机参数，没有飞行试验；所有飞行 sentinel 未猜填 |
| 厂商 Commander/bridge 与 upstream 精确代码身份 | 无厂商二进制/源码对应证据 |
| 实际 Offboard-loss/RC 接管/kill 行为与参数值 | 未验证 COM_OF_LOSS_T、COM_OBL_RC_ACT/override；软件不写这些值 |

这些 UNKNOWN 不影响 source package 被完整审查/重放，但阻止把本次交付说成可带桨飞行的放行。

## 7. Ubuntu build/install 命令

在 Ubuntu22.04 + ROS2 Humble 上解压，进入新源码目录：

```bash
bash scripts/test.sh
bash scripts/install.sh "$HOME/fc01_system/releases/0.3.1-ackfix-20261004T090116Z-Pdwbao/workspace/src/px4_msgs"
cat "$HOME/fc01_system/built-release"
readlink -f "$HOME/fc01_system/current"
```

消息源码路径来自基线记录；路径不同时仅替换为你核对过的同 commit source。安装不会启动服务或切 current。主控源码/build 审查后，在拆桨、未解锁、landed/at_rest、静置 2 s、Offboard 外，另行显式调用新 release 的 update.sh；保存旧 current 路径供同条件回退，完整命令见 README 第3节。

## 8. 第一阶段 props-off、绝不 Arm 的真机 smoke

本节是待执行步骤，不要求用户现在执行。拆掉全部螺旋桨，禁止人工解锁；确认 YAML 中 flight_enabled、flight_actions_enabled、arm_enabled、takeoff_enabled 均 false，flight_parameters_confirmed=false。

只运行一份已验证的 Agent、显式 attitude_altitude controller 和真实交互 console：

```bash
fc01_release="$(cat "$HOME/fc01_system/built-release")"
bash "$fc01_release/scripts/agent.sh"
```

另一个终端：

```bash
fc01_release="$(cat "$HOME/fc01_system/built-release")"
bash "$fc01_release/scripts/run.sh" attitude_altitude
```

第三终端运行 console.sh --json；输入 `status`、`attitude-ready`。仅验收 Agent session → SELF_CHECK → PRESTREAM(>=1.2s) → 合法 Offboard ACK → 较新 nav_state=14/armed=false → OFFBOARD_READY。输出必须 level+捕获 yaw、zero thrust、healthy heartbeat，无 trajectory ownership，无 400/param1=1。

结束输入 quit，停止第一阶段 output，等实际退出 Offboard。**不得继续 Arm/Takeoff。** 不在准备时 echo `/fmu/in/...`，以免触发严格 subscriber 唯一性拒绝；使用节点 JSON/CSV 和 graph info。详细 FAIL 条件、单次 ready lifecycle 和 stop 方法见 README 第4–5节。

## 9. 第二阶段启用前需主控审查

审查真实来源的 hover/min/max/slew、height/velocity/I gains 与积分限幅、高度/速度/倾角边界、ramp/dwell/容差、上升/下降/饱和/任务 timeout；确认 battery/preflight、RC mode/kill takeover、Z/VZ 符号/reset/reference、真实 land/Disarm 和 PX4 Offboard-loss。

全部确认后才讨论打开三门及 parameters_confirmed；本交付不提供猜测参数或安装后的默认起飞命令。当前无 XY 抑漂移，定高不等于水平悬停；actual failsafe 动作仍须硬件证据。

## 10. 完整下载内容

- fc01_system_v0.4.0-attitude-altitude_source.tar.gz
- fc01_system_v0.4.0-attitude-altitude_source.zip
- fc01_system_v0.4.0-attitude-altitude_vs_0.3.1-ackfix.patch
- F_DELIVERY_REPORT_0.4.0-attitude-altitude.md（本文独立副本）
- SHA256SUMS.txt；PACKAGE_VERIFICATION.txt

两个 source archive 内容相同，包含 VERSION/README/src/config/scripts/tools/tests/docs；不包含 git、cache、build/install/log 或编译二进制。patch 可在 pristine r2 上 `patch -p1`，最终归档核验会比较重放后的全部源文件字节与 release。源码包本身不捆绑推测或 main 消息源码，构建时提供指定的已验证 px4_msgs；需要 vendor portable export 时用保留的 export_portable.sh。
