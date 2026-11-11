# 第 7 阶段：原有飞控行为独立复核

本复核只读取生产源码、在私有缓存中编译测试、运行主机替身。未改源码、未提交、未推送、未启动真实 ROS 控制器或 Agent，未连接飞控。所有时间、飞控数据和发布 sink 均为离线可控输入。

## 基准与结果

- 今晚基线：`5b86eeb4094ada009903c0140f66289cfe89411e`（来自已保存的 BASELINE.json，再与真实 Git 对象核对）。
- 本次源码：`4c3c360941575049ebac78dbbf4ce58f098d93b6`，首次核对工作区干净。
- 60 个文件逐项核对全部通过：38 份生产/自研工具 C++ 的完整 token、字面量、预处理逻辑行一致；5 份 `.msg` 声明一致；10 份 YAML 的带类型值一致；5 份配置/冻结备份原字节一致；CMake 执行 token 与 package.xml 语义一致。
- 纯函数生成的当前导航配置与今晚记录一致：目标 1.5 m、模式槽 1、悬停推力 0.5、起飞预留 0.04、最大推力 0.8、程序解锁关闭、通用 flight 入口关闭、RC 动作与起飞门开启、视觉关闭、倾角/时长限制 10 度/30 秒。没有调用配置 validate/preflight 或读取真实设备。

逐文件 SHA、检查方法和结果在 `control_structure.json`。token 一致可以排除控制表达式、分支、浮点顺序、状态更新或消息声明被这次注释整理改动；不证明编译二进制原字节相同，`assert` 错误位置等源码行号也可能变化。

## 新鲜动态差分

`run_differential.py` 重新使用当前驱动，分别编译今晚基线和当前生产实现，未 checkout 或改变工作树。复用了既有 Stage3 驱动，但是将基线显式设为真实的 5b86eeb；结果 JSON 也记录该基线，不能沿用测试注释内的旧 f06715a 标签。

7 个测试方法通过，0 失败、0 错误、0 跳过：

| 范围 | 新鲜证据 |
| --- | --- |
| Core 控制、高度闭环、RC/视觉/恢复 | 14,626 行完整浮点/状态 trace 严格相同 |
| Runtime 请求、ACK 水位、归属和 Unix 会话 | 966 行 trace 严格相同 |
| 发布策略及 target→heartbeat 顺序、异常 | 10,779 行 trace 严格相同 |
| 最终发布三个观察点 | 512 组合及 10 异常变体，523 行 trace 严格相同 |
| 无授权启动、安全事件与 FIFO | 256 启动组合、25 危险事件次序、1,024 FIFO 序列，282 行 trace 严格相同 |
| 真实 SystemNode.tick/status | 9 场景、1,350 个状态样本，1,908 行 trace 严格相同 |
| 真实 SystemNode 人工 RC 全任务 | 707 个状态样本，所有目标/心跳/命令字段逐字节相同 |

707 个全任务状态样本实际走过：IDLE 41、RC_PRESTREAM 80、RC_READY_WAIT_ARM 11、RC_ARMED_WAIT_NAVIGATION 3、ARMED_GROUND 21、THRUST_RAMP 117、ASCEND 131、HEIGHT_HOLD 6、DESCENT 155、LANDED 61、REQUEST_DISARM 3、SAFE_DISARMED 1、SAFETY 77。驱动断言 1.5 m 目标达到；人工 RC 路线仅发出一次 Disarm 命令，不自动发 Arm 或 Offboard 命令；随后经过开关复位、安全地面新证据回到 READY，旧 `flight_started` 不延续。

全任务实际发布顺序为 587 次 TARGET→HEARTBEAT、1 次 TARGET→HEARTBEAT→COMMAND。目标/命令浮点用 hexfloat 编码，未用浮点误差容忍。只将诊断 PID 固定为 4242、过滤与控制无关的启动路径文本；没有省略安全状态或控制字段。`powered-node-*-full.log` 因临时路径长度不同可能不同，真正比较的是有完整状态/发布字段的 `powered-node-*.log`。所有完整原始日志仍保留，SHA 列在 differential_summary.json。

## 真实源码的顺序复核

| 位置 | 代码实际作用 | 本次判断 |
| --- | --- | --- |
| `fc01_system_node.cpp:400` 起 | 先快照与地面证据，再逐个安全事件，再 IPC 请求，再 Runtime 计划，实际发布前重查时限 | 普通健康尾帧没有跳过危险事件观察；有限 trace 与基线相同 |
| `runtime.hpp:371` 起 | 先调用事件观察器，再与旧等待位取逻辑或 | 已发生故障时仍执行后续事件观察，未因短路丢事件 |
| `discrete_event_latch.hpp` | FIFO 有界、溢出粘性，普通 ack 只清队列不清溢出 | 普通缓存刷新不能当作安全锁存复位 |
| `rc_navigation.hpp:252`、`:394` 起 | 地面准备后才承认本轮人工开关边沿、新 armed、新 Offboard；单边消息不能无限续期 | 准备零流与非零起飞权限仍严格区分 |
| `attitude_altitude_core.hpp:1170` 起 | 未取得托管的 armed 拒绝；原始 elapsed 用于停顿判定；接管/健康门早于闭环运算 | RC 接管、Kill、Failsafe 和旧目标失效拒绝未被注释修改 |
| `publication_boundary.hpp:83` 起 | 初查、目标前、心跳前重新取上下文；target 已发后心跳失败抛部分发布异常 | 非原子双发布没有被当作原子成功 |
| `fc01_system_node.cpp:551` 起 | 发布异常锁存边界故障，清输出，禁止对应动作；仅明确的未解锁零输出期限故障可用既有窄恢复 | `PARTIAL_OR_FAILED_SETPOINT_PUBLICATION`、`HEARTBEAT_FINAL_CONTEXT_CHANGED` 是有实际意义的保护 |
| `runtime.hpp:876` 起 | ACK 必须符合真实提交、请求者身份、源时钟、水位；重复 400 歧义保留于进程生命周期 | 没有发现本夜修改导致 ACK 归属变化；并非 MAVLink ACK 自带唯一事务号 |
| `vertical_feedback_guard.hpp:56` 起 | vz-z_deriv 是独立诊断；非法输入、采样回退/大间隔、位置与导数积分残差仍硬拒绝 | 未重新引入未经验证的独立速度差停机；样本不能靠反复旧帧续鲜 |
| `attitude_altitude_core.hpp:2287` 起 | 故障恢复只用严格晚于故障的独立地面样本；不同源分别累计 | 正常全任务可恢复，但下述 C1 时间连续性问题仍未修入生产 |

## ASan/UBSan 新鲜验证

`run_sanitizers.py` 使用当前生产头重新编译 9 个有实际故障/生命周期断言的测试，每个执行 3 次，共 27 次成功，0 跳过。开启 AddressSanitizer、UndefinedBehaviorSanitizer、泄漏检测和遇错退出；没有 NDEBUG。使用非 PIE 的测试二进制减少主机 sanitizer 影子地址冲突，未改变生产编译参数。

测试：`test_rc_navigation`、`test_rc_timing_recovery`、`test_rc_recovery_progress`、`test_vertical_feedback_guard`、`test_offline_timing`、`test_stage3_publication_combinations`、`test_stage3_state_invariants`、`test_overnight_stage6_ipc`、`test_stage3_async_log_resources`。最后一项使用原有仅测试链接包装。IPC 项只发小规模协议/错误数据，不重放 C2 的高负载突发。

编译完整命令、版本、输入 SHA、每次日志/退出码在 `sanitizers/summary.json`。本次没有执行 TSan；由最终汇总记录现有环境限制，不冒称通过。

## 必须保留的限制与候选

在这次有限动态输入及完整生产 token 对照中，没有发现本夜注释/格式整理引入的非预期飞行行为差异。这不代表旧实现不存在缺陷，也不代表真实飞机通过验证。

此前 Stage6 的 C1～C4 仍需在最终报告保留：C1 地面 land 样本的实际新观测跨度不足以覆盖完整窗口；C2 同一轮 IPC 预算仍可能被突发查询放大；C3 原生进程退出与 killpg 间隙竞争；C4 standby 对未成功启动线程 join 的异常路径。候选在隔离证据中，没有应用到生产。尤其 C1 的 READY 提前证据问题未在正常全任务 trace 中暴露，正常 trace 通过不能推翻特定异常复现。

`CONTROL_LOOP_STALL` 仍是停止过期控制计划的保护，C2 能否在 ARM64 真实负载下触发它尚未证明。源时钟/DDS、实际电机推力、PX4 收到并采用目标、真实落地检测、ARM64 时延、气压高度漂移、视觉真实质量均需要后续真实设备和无动力验证。此处 fake ROS sink 返回只证明本地调用完成，不证明 PX4 执行。
