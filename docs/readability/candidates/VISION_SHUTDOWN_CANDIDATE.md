# 视觉适配器 SIGINT 退出候选——尚未应用

本补丁只是独立工具停机候选，**没有写入生产适配器、安装目录或部署包**。本轮中文化与可读性修改维持原有执行行为；不要把候选的通过结果记到原安装工具名下。

## 已确认的现象

本轮全新 ROS 2 Humble 构建成功，安装的 3 个 Python 工具与构建时源码字节一致，5 个消息类型均从全新安装位置导入且通过序列化往返。速度指令诊断工具的真实 ROS 冒烟通过 11 项。

原安装视觉适配器冒烟连续两次失败：正常转换、启动参数只读、双来源撤销、无 `/fmu/in/` 发布者的断言先后执行，最后 SIGINT 干净退出失败。错误发生于 `rclpy` 的订阅 `take_message()`，报 `RuntimeError: Unable to convert call argument to Python object`。脚本整体返回 1，未生成成功报告，**不能写成原安装适配器 5/5 通过**。

把 `053eb2c` 的原始适配器和同行模块精确复制到缓存后，在同一新安装消息、同一当前测试、同一 Domain 171 复现了相同错误。另一次 Domain 173 的原基线测试与适配器组合曾通过。因此这是当前环境中已存在且受时序影响的停机边界现象，现有证据不支持把它归因于中文注释或排版。

## 候选只改什么

补丁仅在 `main()` 中导入 `SignalHandlerOptions`，将 `rclpy.init(args=args)` 改为只安装 ROS 的 SIGTERM 处理器。SIGINT 使用 Python 原生处理器产生 `KeyboardInterrupt`，复用既有 `except`，随后先销毁节点、再关闭仍存活的 ROS 上下文。

候选没有增加 `except RuntimeError`，不会把活跃上下文里的转换错误悄悄吞掉。坐标变换、时间配对、来源数校验、撤销、消息字段、话题、QoS、参数和控制权限均没有改动。SIGTERM 保持原处理方式，本候选只验证了 SIGINT。

## 缓存候选验证结果

- Domain 174、175：各自先确认只有隔离预检节点且没有 `/fmu/` 话题，然后用同一现有 `tests/ros_vision_smoke.py` 测试候选副本；两次均通过全部 5 项，包含双来源撤销和 SIGINT 零状态退出。
- Domain 176：在活跃上下文中给 `rclpy.spin` 注入标记 `RuntimeError`，确认错误传播到调用者，原生 Python SIGINT 处理器仍在，`finally` 关闭了上下文，节点销毁后 `get_name()` 抛 `InvalidHandle`。
- 首次异常探针用 `node.handle is None` 错误判断节点销毁，探针自身失败。Humble 的 `destroy_node()` 不将该属性设为 None；修正为公共操作的 `InvalidHandle` 后通过。首次失败日志也保留，未算作产品失败或掩盖。

## 文件与复现

完整运行目录：`/home/jikl62/.cache/fc01-chinese-20261111/ros/clean-20261111T134428Z-54710`。

同目录 `vision_shutdown_reproduction.json` 记录原安装失败、严格基线复现、候选通过、完整命令文件、日志路径和返回码。`vision_shutdown.patch` 可供以后单独审查；本轮未应用。`candidate_sigint/` 是测试用缓存副本，原工具安装路径仍保持原字节。

这些是 x86_64 本机 Humble 和合成消息验证；没有启动控制器、Agent、串口、systemd 或任何 `/fmu/in/` 发布者，也不能代替 ARM64、飞控或实际飞行验收。两次候选测试记录了当前观测结果，不代表枚举了所有信号时序。
