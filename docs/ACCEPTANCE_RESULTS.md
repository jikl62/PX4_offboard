# 0.4.0-attitude-altitude 验收结果

日期：2026-10-04。实际环境：Ubuntu 24.04 x86_64，g++ 13 / Python3；没有 ROS2 Humble、colcon 或连接的 FC01。验收依据为原交接包 `04_ACCEPTANCE_TESTS.md`，原文保存在 `contracts/`。命令与原始输出见 `TEST_OUTPUT.txt`，基线字节证据见 `BASELINE_INVARIANTS.json`。

## A. 纯 C++ 合同测试

编译与运行入口：`bash scripts/test.sh`；所有测试采用 C++17 和 `-Wall -Wextra -Werror -pedantic`，未使用 NDEBUG。

| 合同项 | 实际证据 | 结果 |
| --- | --- | --- |
| identity/level quaternion，捕获并保持局部 yaw，消除 roll/pitch target | test_attitude_vertical.cpp；真实 q_d wire fixture | PASS |
| quaternion finite/normalized；zero/NaN/Inf/错误 norm 拒绝 | 同上 | PASS |
| disarmed quat reset rebase；armed/active quat reset 停止输出 | vertical + runtime tests | PASS |
| `h=z0-z` / `vh=-vz`；反馈修正方向、hover feed-forward | test_attitude_vertical.cpp | PASS |
| thrust clamp、slew、anti-windup、速度目标限幅、最大高度 | test_attitude_vertical.cpp | PASS |
| stale/invalid Z/VZ、NaN/Inf、Z/VZ reset 拒绝 | vertical + runtime tests | PASS |
| IDLE/SELF_CHECK/PRESTREAM；1.2 s 前没有 Offboard command | test_attitude_runtime.cpp | PASS |
| ACK accepted + 较新实际 Offboard status 的 conjunction；只满足一项不 READY | strict_ready() | PASS |
| 三个配置 gate、参数确认、interactive gate；第一阶段人工 armed 拒绝且停止心跳 | gates()、faults() | PASS |
| wrong-command/target/stale/duplicate ACK；IN_PROGRESS→ACCEPTED；timeout | runtime + 原 ACK lifecycle | PASS |
| serialized command 400 Arm/Disarm；旧 Arm ACK 不满足新 Disarm | flight_and_sequential_400() | PASS |
| 默认锁死的完整飞行状态路径、目标高度 ramp、落地/Disarm ACK+新状态 | 同上；只使用纯反馈 fixture，无真机飞行 | PASS |
| stale attitude、DDS/status/failsafe loss、failsafe、time regression、control loop stall、publisher conflict | faults() 的 12 个故障 fixture | PASS |
| active quat/Z/VZ reset、mode lost、height envelope、excess tilt；超时后 late arm 不恢复 | active_reset_and_timeout() | PASS |
| profile 创建 publisher 的互斥；ground 无 setpoint；attitude mode bits/q_d/thrust_body | ownership + test_attitude_wire.cpp | PASS |
| 地面取消不得增加推力：LAND/断线/shutdown × ARMED_GROUND/THRUST_RAMP/仍在地面的 ASCEND | cancel_on_armed_ground()，9 个 case，含 slow-slew | PASS |
| shutdown 后同批 queued 准备/起飞不再接收；待发 Arm 不发出 | shutdown_rejects_queued_starts()；adapter 发布口人工审查 | PASS |
| interactive console socket 未断开但 PING 超时：第一阶段停止输出；one-shot lifecycle 保留 | preparation_interactive_lease() | PASS |

以上 fixture 中出现的 .4 hover 等数值仅是单元测试输入，未写入飞行 YAML，不代表 FC01 的有效飞行参数。

## B. 基线回归和脚本

| 检查 | 结果及边界 |
| --- | --- |
| 原 `ack lifecycle tests: PASS` | PASS，原测试文件逐字节未改；包括 ground MODE 原事务语义 |
| Runtime::ack()/track()，ground_link.hpp，unix_server.hpp，velocity_core.hpp，protocol.hpp | 与 pristine r2 逐字节相同，PASS |
| Agent/common/update/start/service/collect 与原 ground/flight/sim YAML | 逐字节相同，PASS |
| socket 普通文件不能被替换，timestamp duplicate/regression 不能刷新 freshness | PASS |
| 本机 AF_UNIX bind/listen 和真实 socket ready client | SKIP：sandbox 的 EPERM/EACCES；真实 transport 为 UNKNOWN，未放宽权限 |
| Python tools 测试 | 8 个：7 PASS，1 native AF_UNIX SKIP；真实 CLI wait loop 另用 OS PTY transport fixture 验证 |
| 首次/已有 install 不改 current；显式 update/rollback；release 命名 | PASS，实际文件系统 fixture，外部 ROS/colcon/系统依赖以 fixture 代替；不证明真实 ROS 构建 |
| pinned SOURCE.json vendor 哈希；错误源码拒绝 | PASS |
| 非交互管道、单次 TAKEOFF/VEL/HOLD/LAND 在连接前拒绝 | PASS |
| portable source-only export、动态 0.4.0 名称、无 cache/git/build/binary | PASS，export fixture；实际用户 px4_msgs 导出尚待用户主机执行 |
| 所有 shell `bash -n` 与 Python AST | PASS，数量见 TEST_OUTPUT.txt |
| 完整节点 C++ translation unit 的严格语法 | PASS，使用 release 外的类型/API fixtures；不是 ROS build/link/DDS 验收 |

## C. 构建

指定构建目标：Ubuntu 22.04 x86_64 + ROS2 Humble + pinned px4_msgs。本次实际调用 install.sh 被 Ubuntu 24.04 platform guard 拒绝，且本环境无 ROS2 Humble/colcon，因此**完整真实构建为 UNKNOWN**，没有伪造 PASS。脚本在该拒绝后没有创建 release/current。

可执行入口：

```bash
bash scripts/test.sh
bash scripts/install.sh "$HOME/fc01_system/releases/0.3.1-ackfix-20261004T090116Z-Pdwbao/workspace/src/px4_msgs"
```

实际路径不同时传入已核对的 pinned source。首次或后续安装均只构建新 release；激活必须另行显式 update.sh。

## D/E. 真机 smoke 和第二阶段

第一阶段步骤完整记录在 `../README_zh.md` 第 4 节。必须拆桨、未解锁、三门 false，只到 `OFFBOARD_READY + armed=false + zero thrust`，然后 quit。F 没有执行 0.4 真机 smoke，结果 **UNKNOWN**。

第二阶段软件路径已实现，出厂不可达。所有飞行参数都是 -1 sentinel，parameters_confirmed=false。主控审查和实际硬件验证前不提供默认飞行放行。

## 独立代码审查

代码在 f10801c 获得 source delivery approval；没有未关闭的 Critical/Important/Minor finding。三类初审问题均有 RED/GREEN 回归，并修复 slow-slew 在 ASCEND 仍未离地的扩展 case。审查没有声称真实 ROS、ARM64、native socket 或飞行测试通过。详见 `CODE_REVIEW.md`。
