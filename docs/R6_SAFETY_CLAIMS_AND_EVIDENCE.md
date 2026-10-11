> 历史 R6 基线记录，保留原结论与来源。本文件不是 R6.1 的当前结论；请先读根目录 `README_FIRST.md` 和 `docs/R6_1_SAFETY_EVIDENCE.md`。

# R6 安全主张与证据边界

**结论：允许继续离线开发和 C 复审；本报告不批准真机无桨控制命令、台架或自主飞行。**

## 控制权状态表

| 当前情况 | R6 动作 | 输出/责任 | 终态证据 |
|---|---|---|---|
| 健康 Armed+Offboard，普通下降超时 | 一次进入 RecoveryDescent，重定向下降目标 | 同周期及之后限幅/slew 双 sink，期限有限 | 需要原 ground/disarm 证据或后备模式新 status |
| 原合同验证且当前完整本地位置有效 | 一次请求 guarded Land21 | PX4 仍 Offboard 时继续受监督输出 | ACK 与模式分开；任务能否运行仍需外部证据 |
| 21 ACK 接受，仍 Offboard | 不停流、不重复 21 | 同一绝对恢复预算 | ACK 不能当作交权 |
| 更新 post-commit status 已离开 Offboard | 停止 Companion 输出，锁住 Authority | 观察到模式退出，控制能力未验证 | 不是安全着陆证明 |
| 恢复总预算耗尽，仍 Armed+Offboard | Safety，无新 Arm/Disarm/21 | PX4_FAILSAFE_PENDING / UNKNOWN_AIRBORNE | 外部后备未证实：P0 RED |
| Z/VZ stale/reset、姿态坏、调度 stall、DDS 图失效、Kill、部分 sink 失败 | 不伪造控制、不猜零推力 | 明确不确定或 emergency kill | 独立 PX4 后备待验证 |
| SIGINT/SIGTERM | 请求原 LAND 恢复、阻止新任务，预算不可续期 | 仅健康恢复时继续双 sink | 纯 Runtime OS fixture，非真实 rclcpp 证据 |
| SIGKILL/崩溃 | 进程无法继续控制 | 只能依赖独立 PX4 后备 | 本轮没有实机/PX4 后备证明 |
| 新 disarmed ground 数据连续驻留 | 只读观察 safe ground terminal | GroundDisarmedConfirmed，可停止恢复 | 新 status + 连续新 land + Z/VZ/at_rest，无 ACK 冒认 |

## 离线证据

原始 R5 `scripts/test.sh` 在本环境退出 0；原生 Humble 不在此环境。
新增 R6 timeout 正向测试先在 R5 复现断流，再在 R6 验证限幅、slew 和双 sink。
最终完整回归结果、每项退出码和原始输出见交付 `evidence/`；成功数以日志为准，
AF_UNIX 两项环境跳过不得记为通过。

最终 `scripts/test.sh` 退出码 **0**：21 个 C++ 回归可执行文件完成；
Python 30 项中 **28 PASS / 2 SKIPPED**。实际 Node 的 API fixture 语法检查通过。
原 C11/C12 漏洞探针的预期 abort 仅作历史重放，正向安全断言是独立的 R4 ACK tests。

| 项目 | 范围与解释 |
|---|---|
| 原 C-S10 timeout/shutdown | R5 RED；R6 原 460 tick 窗口 GREEN，文件未修改 |
| 新延长 ownership gate | R6 **RED / exit 1**；38.05 s 预算耗尽，防止把延迟误当问题已关闭 |
| 21 ACK/当前位置/最终 guard | 合成状态正反例；仅证明实现，不证明 PX4 执行 |
| Command400 / C11-C12 | 原字节不变；肯定式 R4 tests 验证跨事务 ACK、state-only 终态、反弹无重试 |
| 1Hz / freeze / bounce / gap | 原 R5 联合判据保持；旧 B timeout 预期透明调整，原副本存档 |
| 部分 target/mode 提交 | 真正调用 C++ boundary，再注入异常；分别计数，不宣称原子性 |
| OS signals/crash | 本地 C++ Runtime fixture 收真实 OS 信号，外部 PX4 恢复 UNVERIFIED |
| Node adapter | 实际源码通过窄 type/API fixture syntax compile；非 ROS 编译/链接或 DDS 验证 |
| CPU 移植 | C++17、标准数值/steady_clock、Python；未执行 aarch64 构建 |

## 尚未关闭的 P0

1. 恢复预算耗尽或不可控制后，当前 FC01/PX4 的独立后备是否实际运行，未知。
   Position 名称不能证明有 XY；Land21 默认关闭。没有合格后备时自主飞行 NO-GO。
2. 本版本没有真实 ROS2 Humble native build/link 和真实 DDS 验证。
3. 真实飞行器 hover/slew/动力/电池、估计器、遥控接管/Kill 未有本轮证据；默认动力参数 unset。
4. 正式智能体 C 未签署针对本完整源码/哈希的分项结论；开发代码审查不能替代它。

## 官方资料核对（2026-10-09 网页读取）

- [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)：
  ROS2 的心跳是 OffboardControlMode，与姿态设定值分开；失流后的动作受参数和能力影响。
  态度 Offboard 的文档要求不能被当作自动 Land/Position 后备可用性的证据。
- [PX4 v1.16 Land](https://docs.px4.io/v1.16/en/flight_modes_mc/land)：
  文档要求有效本地位置，空中无相应估计不能据模式名认定可切入；可用 Command21 请求。
  厂商固件实际行为仍必须独立验证，本轮没有假定无 XY 的 Land 可以工作。
- [PX4 v1.16 Altitude](https://docs.px4.io/v1.16/en/flight_modes_mc/altitude)：
  属人工操纵定高路径，不把它自动当作无需操作员的自主后备。

本轮未连接 FC01、未发送真实 Arm/模式/参数、未飞行。最终 sink 的“commit”只表示本地
publish 返回；所有 actual PX4 task、接收及物理安全均保留 UNKNOWN。
