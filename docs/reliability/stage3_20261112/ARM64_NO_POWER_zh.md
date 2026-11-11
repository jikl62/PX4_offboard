# Orange Pi Zero 3W：下一阶段无动力验证准备

当前适合推进隔离、无动力 ARM64 构建验证。尚无 ARM64 编译或性能通过结论，不能据此运行实机控制任务。本轮没有访问 Orange Pi。

环境需要 ARM64 的兼容 Ubuntu/ROS 2 Humble、C++17、CMake、colcon、Python 依赖及工程已固定版本的 px4_msgs。先核对架构、RAM、可用磁盘和 ROS/Python ABI；源码里的 Msgs 校验仍必须通过，不能替换成随意最新 PX4 消息。

建议在独立临时构建目录只运行纯主机断言程序，单作业控制内存峰值：

```bash
cd /path/to/FC01_source
uname -m
free -h
df -h .
cmake -S native/tests -B /tmp/fc01-stage3-arm64-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01-stage3-arm64-tests -j1
ctest --test-dir /tmp/fc01-stage3-arm64-tests --output-on-failure -j1
```

这些 C++ 测试只使用合成反馈、临时 socket 和内存 sink。日志资源注入测试依赖 Linux/GNU 链接与 ELF 插入，须在 ARM64 上实际编译确认，不能把 x86_64 结果冒充支持。还应跑 Python 根目录和 native/tests 回归；历史差分需要保留 Git 基线对象。

ROS 包可在独立 cache 下单作业 colcon build，复用已经核验的 px4_msgs overlay；只编译，不执行 navigation_deploy、navigation_service、standby 或 fc01_system_node，不激活 systemd。具体依赖/overlay 绝对路径应按目标机实际安装填写，不能照抄开发主机 cache 路径。

先做无图序列化/纯接口基准：source Humble、固定 px4_msgs 和刚构建的本工程 overlay 后运行：

```bash
python3 tests/benchmark_vision_interfaces.py --samples 3000 --output /tmp/fc01-stage3-arm64-performance.json
```

记录编译峰值 RAM、RSS、CPU/温度/降频、p50/p95/p99/max；同步操作耗时不能替代完整 20 Hz 控制循环 deadline。若做真实 ROS 合成通信，先设置非生产 localhost-only 域并预检没有 /fmu 端点，使用仓库只读 smoke；不要在有实机导航服务的同一域注入合成状态。

后续需要隔离域高频/延迟/乱序、真实录包、低磁盘/日志慢端的受控测试，保留原安全阈值；不能靠放宽 stall/source TTL 让压测通过。系统资源不足时记录失败与降载结论。第一次上板的目标是构建及无动力验收，不含解锁、电机或动力试飞。

当前未知：板型实际 RAM/系统与内核、RMW 和 PX4 消息版本、热/存储负载、真实视觉时钟/协方差/尺度、XRCE 重连、PX4 采用和 failsafe。真实 PX4 反馈恢复必须用同程 ULog/导航报告核对；主机信号测试不能证明飞控已进入预期后备模式。
