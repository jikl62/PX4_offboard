# Orange Pi Portability Contract（供 F 编码时遵守）

最终平台：Orange Pi Zero 3W / Allwinner A733 / ARM64。

当前阶段不要求安装 Orange Pi 环境，但 0.4 源码必须避免以后重写。

## 硬要求

- C++17 + rclcpp + ROS2 Humble；
- aarch64 可原生编译；
- 禁止 x86-only 二进制依赖；
- 禁止 Qt/QGC/RViz/Gazebo 成为运行依赖；
- 禁止硬编码 `/home/jikl62`；
- 路径通过 `$HOME`、ROS package share、参数或环境变量；
- 参数全部 YAML；
- 串口路径可配置，运行脚本优先 `/dev/serial/by-id/*`；
- Micro XRCE-DDS Agent 版本保持与 PC 基线一致，不在代码里假设系统全局库；
- headless 可运行；
- 不依赖键盘才能保持安全；
- 日志路径可配置且支持无桌面环境；
- start/stop 过程不得自动 Arm/Takeoff；
- systemd 化留接口，但 F 此阶段不要安装 systemd 到用户机器。

## 推荐代码边界

建议拆成可单测的纯 C++ 核心：

- `attitude_altitude_core.hpp/cpp`
- `vertical_controller.hpp/cpp`
- `attitude_reference.hpp/cpp`
- `safety_limiter.hpp/cpp`
- `command_transaction`（复用/扩展现有 0.3.1）
- ROS adapter/node

不要把控制数学、ROS callbacks、文件路径、operator CLI 全塞进一个源文件。

## Orange Pi 手册

本包附 `manuals/OrangePi_Zero3W_A733_manual_v1.0.pdf`。F 只需做 portability audit；不要替用户刷系统/装环境。
