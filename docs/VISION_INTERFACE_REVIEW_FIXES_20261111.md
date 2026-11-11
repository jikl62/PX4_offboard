# 视觉接口审查修复（2026-11-11）

这次修复针对提交 `8b57077` 之后只读审查确认的五处问题。修改限定在新增视觉诊断、状态校验、离线预览和对应测试；`package.xml` 补充诊断节点直接使用的 `rcl_interfaces` 运行依赖。自动起飞与飞控发布路径没有接入这些模块。

| 问题 | 修复后的行为 |
| --- | --- |
| ROS 参数修改成功但实际仍用旧值 | 自定义诊断参数均声明为启动时配置、运行期只读；启动 `--ros-args -p` 覆盖仍有效。运行中修改会明确失败，查询值与实际配置保持一致 |
| 正常新包吞掉旧候选的源年龄过期 | 接收回调和快照共用候选失效检查；替换缓存前记录源年龄失效，恢复后计数和异常证据保留 |
| 非法 epoch 吞掉旧状态超时 | 在本机时钟有效时先检查旧缓存到期，再处理新消息身份；退役 reset、未知来源和非法身份都不能跳过这项检查 |
| Ctrl+C 退出丢失节流中的异常 | 退出前强制写出待处理异常并保存终态快照；不查询已关闭的 ROS 上下文，单项日志失败仍尝试保存快照 |
| 极大有限数经过限幅变成有效零加速度 | 在世界目标限幅前、速度差计算后和加速度限幅前检查分量及范数；计算溢出明确拒绝，普通速度和限幅方向保持原行为 |

参数改动只适用于独立诊断进程，不影响生产控制器的参数行为。需要改变诊断 Topic、QoS、阈值或输出目录时，以新参数重启诊断进程；这个操作不清除飞机故障或改变 RC 权限。

正常快照增加 `receiver_running=true`。终态快照为 `receiver_running=false`，`cache_usable=false`、`current_candidate=null`，并在 `cache_errors` 中包含 `RECEIVER_STOPPED`。进程突然断电或被 SIGKILL 时无法保证最终落盘；保留文件仍须结合进程状态和记录时间判断。

退出记录用 `stopped_at_monotonic`、`stopped_at_system_ns` 保存本机停止时间，`stopped_at_ros_ns` 保存最后观察的 ROS 时钟，并标注 `shutdown_clock_source=last_observed_ros_clock`，避免模拟时钟被系统时间替代而制造伪失效。待处理异常强制刷新一次；写入失败保留待刷新数据供重试，并报告错误，不替换原始运行异常。

对应新增回归覆盖“源年龄过期→健康/非法尾帧”“接收或源年龄过期→退役/非法 epoch→健康恢复”“时钟回退不伪造到期”“强制刷新与文件错误”“启动配置覆盖及运行时参数拒绝”“真实 SIGINT 终态落盘”，以及中间减法、乘法、范数溢出和正常限幅行为。

## 本轮验证

验证记录及源码摘要见 [本轮机器可读证据](evidence/20261111_vision_review_fixes.json)。原开发记录 [VISION_INTERFACE_VERIFICATION_20261111.md](VISION_INTERFACE_VERIFICATION_20261111.md) 和原证据保留，表示此前版本的结果。

| 本轮验证 | 结果 |
| --- | --- |
| 诊断接收器 Python | 41 项通过 |
| 状态校验 Python | 41 项通过 |
| 原起飞配置、视觉关闭兼容和导航服务 Python | 45 项通过 |
| 载荷与构建证据 Python | 22 项通过 |
| 原起飞、RC、Kill/Failsafe、权限及发布安全 C++ | 8 个程序通过 |
| 新增离线预览 C++ | 458 个检查通过，严格编译及 UBSan 通过 |
| 安装后的真实 ROS 通信 | 1 组通过，包含 11 个检查项 |
| ROS 2 Humble 增量构建与安装 | 通过，未见编译警告 |

Python 共 149 项通过，每个用例只计一次。接收器在补充退出时钟域处理后重跑；其余未受影响测试未重复累计。安装后的三个 Python 工具逐字节匹配源码，新增直接运行依赖已安装。

原始构建与运行日志使用临时目录，未打入交付源码；最终整理时早先的临时产物已被清理。本记录保留执行时记录的结果和摘要，包装回归在最终整理时重新生成日志。不能将日志路径视为长期归档。

本轮测试命令：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_velocity_command_diagnostic test_vision_state_validation
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_takeoff_configuration test_vision_settings test_navigation_service
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  test_vertical_diagnostic_bundle test_navigation_build_evidence
```

C++ 严格编译并运行新增预览测试及以下原有程序：`test_rc_navigation`、`test_attitude_runtime`、`test_authority_runtime`、`test_rc_before_ready_recovery`、`test_receive_schedule_safety`、`test_m1_safety`、`test_publication_runtime`、`test_visual_runtime`。新增预览另运行 UBSan，使用 `-fsanitize=undefined -fno-sanitize-recover=undefined`。

真实 ROS 冒烟程序使用安装后的诊断脚本，在 localhost 隔离 Domain 193 内运行；除既有类型、QoS、失效与恢复检查，还通过参数服务逐项查询只读描述、尝试修改参数并确认拒绝，然后以较长快照/日志周期构造退出前尚未落盘的异常并发送 SIGINT。测试只启动合成发布者和独立诊断，不启动控制器、Agent、串口或电机。

## 原工程保护

对照本轮基线 `8b57077` 逐字节核对 65 个原工程文件，包括 Runtime、节点、RC/权限/高度控制与保护头文件、原 Odometry adapter、正式配置和导航服务入口。修改的预览头文件不被生产 Runtime 或节点包含。

继续保留：`visual_navigation_enabled=false`、目标高度 `1.5 m`、`aa_hover_thrust=0.50`、`aa_takeoff_reserve_thrust=0.04`、`aa_thrust_max=0.80`。RC、Kill、Failsafe 和原有恢复流程保留。本轮验证是主机软件回归，没有进行实机起飞或 ARM64 验收；`dist/` 安装包没有重新生成。
