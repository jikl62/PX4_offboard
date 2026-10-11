# Orange Pi portability audit

依据交接的 05 合同，以及 OrangePi Zero3W A733 手册的产品规格、3.8 SSH、3.12 USB、3.27.1 Ubuntu22.04/Humble、3.29.3 Jammy 编译器章节；FC01 手册的串口映射、固件及 RC 输入章节已读取。只做审查，没有安装系统、联网 SSH 操作或 systemd 安装。

| 约束 | 源码证据 | 结论 |
| --- | --- | --- |
| C++17/rclcpp/Humble | CMake cxx_std_17；依赖 ament_cmake/rclcpp/px4_msgs | 源码 PASS；真实 Humble build UNKNOWN |
| ARM64 native build | deploy 接受 aarch64/x86_64；无 SIMD/架构专用代码 | 源码 PASS；A733 运行 UNKNOWN |
| 无 Qt/QGC/RViz/Gazebo runtime dependency | package.xml/CMake 仅上述依赖 | PASS |
| 无用户固定 home | 路径由 HOME/FC01_HOME/FC01_SOCKET/ROS 参数/env 提供 | PASS |
| YAML 参数 | aa 控制、准备、限制/门限均有 YAML；flight sentinel 禁止隐式默认飞行 | PASS |
| 串口可配置/by-id 优先 | 基线 common.sh + FC01_DEVICE；不自动选择多个串口 | PASS |
| 基线 Agent 版本和私有 deps | build_agent.sh 未改，v2.4.3，deps/lib/lib64 | PASS；实际已安装 Agent 与脚本版本一致性待用户主机核对 |
| headless/无键盘保持安全 | 单进程监督、heartbeat health gate、console lease 失效下降/释放 | 源码/核心测试 PASS；PX4 takeover 实机 UNKNOWN |
| 配置日志路径 | csv_path、FC01_LOG_DIR，stdout JSON | PASS |
| start/stop 不自动 Arm | idle startup；stop 需地面未解锁；默认三门 false | 核心测试 PASS |
| 服务接口但不自动安装 | 保留基线 service.sh/start.sh；installer 不调用 service.sh | PASS |

手册描述 Jammy/Humble、USB host 与 SSH，支持该迁移路线的条件判断，不证明你的具体板卡已配置正确。跨 ARM/x86 迁移需重新原生 colcon build；不得拷贝 PC 编译二进制代替。

安全上不依赖键盘持续提供“健康”标签。数据不可信或控制循环停止时，setpoint 与 heartbeat 同时停止；这将触发 PX4 的实际 Offboard-loss 逻辑，具体动作必须在第二阶段前核对。SSH 交互 console 使用 TTY，单次只读/地面命令不需要 TTY。
