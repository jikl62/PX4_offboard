# FC01 0.4.1-safety-hardening source release

这是从已冻结 `0.4.0-attitude-altitude` 单独复制、修改的离线安全加固候选。仅闭合 D 的 I-01 最终 publisher 边界和 Unix socket 安全诊断/更新逻辑；不改变四个 gate=false、飞行参数=-1 UNKNOWN 的交付配置。

**本交付不授权真实 Arm、Takeoff、非零推力或飞行。** 用户真实 current 和 0.4.0 没有被修改。下一次真机第一阶段 smoke 仍按总部已冻结的 0.4.0 安排；不要因为下载或构建此包就切换到 0.4.1。

## 改动和行为

- Arm 最终发布点统一检查 profile=attitude_altitude、四 gate、REQUEST_ARM、shutdown=false、连接和 live interactive lease、disarmed、fresh/valid attitude/Z/VZ/status/land/failsafe、ownership、ground/reference、电池/RC/preflight。
- 第一阶段 PRESTREAM/REQUEST_OFFBOARD/OFFBOARD_READY 仍能在四 gate=false 时发布精确 `[0,0,0]`。上游有限非零值（包括 double 极小值及侧向分量）被拒绝、锁存故障，绝不偷偷归零并继续。
- 非零推力仅允许 THRUST_RAMP/ASCEND/HEIGHT_HOLD/DESCENT/LANDED，四 gate 和有效飞行参数、armed/offboard、health/reference/envelope 必须通过。其他阶段必须零推力或停止输出。
- attitude target 和 heartbeat 只有同一个受监督函数能够发送；拒绝时两者均停止，JSON 输出 publication_block_reason/publication_fault_latched。新增 flight_parameters_confirmed、heartbeat_enabled、attitude_output_enabled；原有 JSON 字段保留。
- 普通 wire() 仅负责字段编码；它本身不是发布许可。所有真实 attitude ROS sink 都经 publication_boundary.hpp，所有 Arm 400/param1=1 都经最终 command sink guard。

两项有意收紧：任何 flight/sim profile 的 Arm 最终发布都被拒绝（满足“任何 Arm 必须 attitude_altitude”的本次合同）；shutdown 锁存后态度目标/heartbeat 全停，不能继续非零下降，交还 PX4/operator。第二阶段断线后的健康 DESCENT/LANDED 仍可按既有状态机受控下降；不允许失去 lease 后继续 ASCEND/HOLD。

## 离线验证

```bash
bash scripts/test.sh
```

该脚本运行原 ACK/lifecycle/数值/wire/socket/工具测试，加上最终边界、runtime→boundary 和 socket/更新回归；另用窄类型/API fixture 编译真实 ROS adapter。fixture 检查不是 Humble build/link 或 DDS 证明。

本执行环境 Ubuntu24.04 无 ROS/Humble/colcon；0.4.1 真实构建为 UNKNOWN。native AF_UNIX socket() 被环境禁止，相关测试明确 SKIP/UNKNOWN；真实 socket 类型 inode 的保留以及 advisory flock/文件系统更新测试实际执行。

## 版本和构建

VERSION=`0.4.1-safety-hardening`；package.xml=`0.4.1`；px4_msgs 固定 `392e831c1f659429ca83902e66820d7094591410`，PX4 v1.16。不得用 main 代替。

以后在 Ubuntu22.04/Humble 上，用已经核验的同 commit 消息源码构建候选：

```bash
bash scripts/test.sh
bash scripts/install.sh /absolute/path/to/verified/px4_msgs
```

install.sh 保持 0.4.0 原文：建立唯一新 release，构建成功才写 built-release；从不切 current、启动节点/Agent或服务。此处没有要求现在安装、激活或连接飞控。

## Socket 和显式更新

0.4.1 不再根据 ECONNREFUSED 删除已有 socket。UnixServer 启动和 update 都对已有 socket 路径采取诊断后拒绝；普通文件、符号链接（含 dangling）、FIFO、意外类型/owner、未知服务状态也拒绝，原对象保留。

新 controller 在整个生命周期持有 `control.sock.lock` advisory flock。新 update 使用同一把锁，锁住直到原子 current 切换结束，再次检查锁的 dev/inode、socket 路径、service inactive。锁文件永久保留，**不得人工删除 .lock 文件**。该机制只协调遵守约定的 0.4.1 进程；旧 0.4.0/任意同 UID 外部程序不遵守锁，不能声称对它们提供原子安全证明。

更新前必须由主控确认真实飞机在安全地面、disarmed、Offboard 外，并显式停止所有旧服务/手动 starter，防止更新时重新启动。本版本不再连接 console 执行 update-check/stop，不自动 stop/start 服务；这样诊断不会触发会话断线的控制语义，也不会自动恢复任务。

诊断命令（只检查文件系统，不连接节点）：

```bash
python3 tools/socket_safety.py "$FC01_SOCKET"
```

使用时明确设置实际 FC01_SOCKET，或传入已核验的绝对 socket 路径。发现 SOCKET_PRESENT_LIVENESS_UNKNOWN 时更新保持中止，打印 owner/type/inode 相关诊断和 shell-quoted 手工 rm 命令。仅在独立确认无 listener、全部 starter 已停止且不会重启、重新核对 owned socket 类型及 inode 后，才由人手工清理该一个 socket。ECONNREFUSED、pgrep为空或 service inactive 中任何单项都不是删除许可。程序不会执行打印的命令。

候选经独立源码/build 审查后，另行显式激活/回退（本次未执行）：

```bash
bash /absolute/path/to/new-release/scripts/update.sh /absolute/path/to/reviewed-release
```

目标必须是 FC01_HOME/releases 下已经构建的 release。不存在 socket 且 systemctl 明确 inactive 才切换；live/starting/stale-looking socket、active/unknown service 和竞态均拒绝。成功后不会启动任何控制进程。

## 保留与 UNKNOWN

Runtime::ack()/track()、ground transaction、target=1/191、from_external=false 接纳、freshness/watermark/IN_PROGRESS/5秒超时不变。attitude_altitude 仍只有 Mode+Attitude，没有 Trajectory publisher。旧报告位于 docs/，属于历史 0.4.0 证据，当前结果以 docs/G_DELIVERY_REPORT.md、G_ACCEPTANCE_RESULTS.md、G_TEST_OUTPUT.txt 为准。

真实 0.4.1 Humble build、DDS/QoS/频率、FC01 READY/零推力接收、Offboard-loss/RC 接管、Orange Pi 运行及任何物理飞行均 UNKNOWN。0.4.0 的用户成功 build 不能替代新版本 build。两个 DDS 消息不是原子事务；异步信号、调度、传输和 failsafe 的实际行为仍需独立验证。
