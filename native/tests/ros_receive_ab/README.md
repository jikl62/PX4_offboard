# 真实 Humble 接收积压对照（主动选择运行）

这是独立的普通消息实验：固定 localhost、Domain 188、`rmw_fastrtps_cpp`，不使用 `/fmu`、Agent、串口或飞行控制器。仅合成消息发布器属于本实验；根目录 `feedback_age_probe.py` 是另一个供现场并行使用的被动观察工具。

```bash
source /opt/ros/humble/setup.bash
cmake -S native/tests/ros_receive_ab -B .build/ros_receive_ab
cmake --build .build/ros_receive_ab -j2
python3 native/tests/ros_receive_ab/run.py --build-dir .build/ros_receive_ab \
  --output .build/ros_receive_ab/results.json
```

默认 25 Hz 生产、20 Hz 消费调度、BestEffort/KeepLast(10)，总计约 35 s。可用 `--input-hz 20.8` 或 `40`，`--seconds 4..30`。每种方式保持相同的 50 ms 单线程控制周期：原 free `spin_some`、persistent `spin_some`、`spin_all(5ms)`、最多 10 次/5 ms 的完整 `spin_some` pass。无回调则立即停止额外 pass。生产安全事件的停止逻辑由工程单元测试验证，本实验只有普通数据消息。

输出包含消费数/频率、每周期回调数、源年龄、原始 RMW 接收戳差、pass 数和耗时。另验证 fresh envelope 无法让 600 ms 旧 raw sample 通过生产 `source_sample_in_clock`，以及 12 ms 回调会超过名义 5 ms 协作预算。预算不会抢占回调，不能视为最坏执行时间保证。

Humble rclcpp 16.0.19 的 `spin_some_impl(..., false)` 一次收集后不重复收集，已选择的 subscription handle 被移出 ready 集。`spin_all(..., true)` 重新收集；无数据时立即返回。[executor.cpp](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/src/rclcpp/executor.cpp#L416)、[memory strategy](https://github.com/ros2/rclcpp/blob/16.0.19/rclcpp/include/rclcpp/strategies/allocator_memory_strategy.hpp#L246)。

初次本地复现中，25 Hz 输入使用原方法仅消费约 20 Hz，源年龄增长至约 368 ms；重复 pass 消费约 25 Hz，源年龄保持一个采样相位内。KeepLast 满后丢弃旧消息会使年龄增长呈平台，不能把平台当成恢复。具体数值取决于机器和调度，真实 ARM64 表现须另行验收。

现场探针另用以下命令，不更改 Domain/RMW 环境，只读现有两个反馈话题及状态文件，每 5 s 记录一条 JSON：

```bash
source /opt/ros/humble/setup.bash
# 此处再 source 当前控制器使用的 px4_msgs workspace。
python3 feedback_age_probe.py --duration 180 --output "$HOME/feedback_age_probe.jsonl"
```

`--output` 文件须不存在；默认读取 `~/FC01_navigation/status.json`，可用 `--status` 指定。探针连续 `spin_once`，单次等待不超过 10 ms；不按固定 100 Hz sleep 限制实际消费。保留原 envelope/sample 和 RMW 戳、符号及缺失值，30 s 单调时间窗口拟合年龄增长。`rmw_to_callback_s` 默认标为 epoch 未核实的诊断差值，包含接收后多个软件环节，不能直接当作纯排队时间。状态文件和探针取样时刻不同，需结合原始时间戳及记录年龄比较；工具不会据此宣称恢复安全或允许起飞。

探针为 ROS 2 Humble 编写：标准 Python executor 丢弃 take_message 返回的元数据，因此它在本进程内保留该 tuple 的两个元素；不修改 ROS 安装包。标准 parameter_events 元数据仍由 rclpy 创建，rosout 和参数服务关闭，不创建 `/fmu/in`、运动目标或 IPC 发布入口。

探针逻辑回归与隔离的 5 s 真实 ROS 检查：

```bash
PYTHONPATH=.:tests python3 -m unittest test_feedback_age_probe -v
# 真实检查先 source 当前 px4_msgs workspace；固定 localhost Domain 189。
python3 native/tests/ros_receive_ab/probe_smoke.py --report .build/ros_receive_ab/probe_smoke.json
```

真实检查用合成 40 Hz 状态、600 ms 旧 raw sample，验证探针保留调用环境 Domain、取得原 RMW 接收戳、原始样本仍旧、状态文件未变，以及 ROS graph 中探针仅有标准 parameter_events 发布元数据。它不会启动导航控制器或 Agent。
