# 视觉接口交接与源码阅读路线

本说明对应队列 5/7。当前真实工程的视觉导航保持关闭；收到视觉消息不等于获得 Arm、Offboard、起飞或最终设定值发布权限。这里的接口测试使用合成数据，没有测量真实相机、SLAM 或板上性能。

## 先区分“飞机实际怎样”和“希望飞机怎样”

| 类型 | 含义与来源 | 当前去向 |
|---|---|---|
| `nav_msgs/msg/Odometry` | 视觉估计的传感器/child 原点实际位置、姿态、线速度、角速度；不是导航目标 | 显式标定后的状态适配器 |
| `jikl_fc01_system/msg/VisionHealth` | 同一测量的跟踪、尺度、来源、时钟及标定声明 | 与 Odometry 的整数纳秒 stamp 完全相等才配对 |
| `jikl_fc01_system/msg/VisionState` | 转成 FC01 坐标和机体原点后的实际状态，有效位说明哪些数值可用 | C++ Inbox；生产节点只有启用视觉时才创建该订阅 |
| `jikl_fc01_system/msg/NavigationCommand` | 既有视觉外环的模式、绝对点或归一化杆量请求 | 既有视觉控制器，仍经过原飞行权限和最终发布检查 |
| `jikl_fc01_system/msg/VelocityCommand` | 视觉制导希望实现的机体 FRD 速度，单位 m/s | **只读诊断器**；本轮没有接到生产 Runtime，也没有自动转成 NavigationCommand |
| `jikl_fc01_system/msg/AttitudeThrustPreview` | 离线/诊断计算的姿态推力预览 | 不是 PX4 设定值，不授予发布权 |

例如实际 `VisionState.velocity=[0,0,0]` 表示估计静止；目标 `VelocityCommand.vx=0.2` 表示希望机头方向前进 0.2 m/s。不能把后者写进实际速度，让闭环误以为飞机已经运动。

`NavigationCommand` 的 HOLD 在进入时捕获位置；CRUISE 的 `position` 是世界系绝对点；FOLLOW 的 sticks 是 `[-1,1]` 的请求，不能直接解释成 m/s；LAND 仍须检查命令来源、时间和有效性。它的 reset 绑定 VisionState 的估计周期；VelocityCommand 的 reset 属于独立目标生产方，含义不同。

## 在 VS Code 中沿这条路径读

1. [五份消息声明](../../../../native/src/jikl_fc01_system/msg/)：先看字段、单位、有效位与默认值。零值默认构造不证明测量有效。
2. [vision_odometry_adapter.py](../../../../native/tools/vision_odometry_adapter.py)：`AdapterConfig` → `convert_odometry` → `StampPairBuffer` → `GuardedOdometryAdapter` → `main`。前几层可以离线测试，`main` 才创建 ROS 端点。
3. [vision_ros_adapter.hpp](../../../../native/src/jikl_fc01_system/include/jikl_fc01_system/vision_ros_adapter.hpp)：把 ROS 字段映射到内部结构；缺少有效位的物理量保持未知，不替上游猜外参。
4. [vision_navigation.hpp](../../../../native/src/jikl_fc01_system/include/jikl_fc01_system/vision_navigation.hpp)：Inbox 保存状态、目标和撤销水位；Controller 检查当前配置与实际反馈，再计算外环目标。
5. [velocity_command_input.hpp](../../../../native/src/jikl_fc01_system/include/jikl_fc01_system/velocity_command_input.hpp) 与 [只读诊断器](../../../../native/tools/velocity_command_diagnostic.py)：独立的目标速度接收合同及预览，当前不接管原飞控。

Python 的 `self` 表示当前实例；`AdapterConfig` 集中保存本次转换合同，`frozen=True` 阻止属性重赋，并不深度冻结外部传入的可变容器。实际 ROS 加载路径将旋转和平移数组转成 tuple。C++ 的 `const &` 接收只读引用，避免复制结构；输入引用不代表调用者可以把状态对象当成权限令牌。内部对象保存的历史水位与当前数据有效性各有作用，不能仅凭“最后一帧健康”删除中间撤销事件。

## 坐标、原点与单位

世界系 ENU 为东、北、上；FC01 内部 NED 为北、东、下。只做轴约定转换时 `[1,2,3] ENU → [2,1,-3] NED`。机体系 FLU 为前、左、上，FRD 为前、右、下，所以同一向量的 y、z 变号。轴约定转换不等于完成相机安装标定，也不等于知道真实北向。

Odometry 的 pose 属于 `header.frame_id` 世界系，twist 属于 `child_frame_id`；不能直接把 child 线速度当作世界速度。ROS 四元数字段通常按 x/y/z/w 存放，FC01 数组按 Hamilton **w/x/y/z**，表示机体 FRD 到内部 NED 的旋转。

适配器显式使用四组配置：轴约定、传感器到机体的旋转、机体系杆臂、世界对齐旋转和平移。设 `r` 为机体原点指向传感器的 FRD 向量（m），`R` 为机体到内部世界的旋转：

```text
机体实际位置 = 转换后的传感器位置 − R r
机体实际速度 = 转换后的传感器速度 − R (机体系角速度 × r)
```

非零杆臂时，旋转会让相机原点产生额外线速度；缺少可信角速度及其来源时不能假造零值完成修正。本轮增强的组合测试同时施加世界对齐、安装旋转和非零杆臂，检查位置、姿态、角速度与修正后的线速度 `[-3,0,4] m/s`。

NED 的 `z=-1.5 m` 只是位于视觉原点上方 1.5 m，不能直接理解成距地高度或本次起飞平台。`vz=-0.2 m/s` 是向上运动；原 1.5 m 起飞和原地面参考逻辑未在本轮改变。

## 时间、有效性和重定位

测量 stamp 是**上游测量时刻**；本机 monotonic 接收时间用于等待窗口和接收 TTL。消息刚到不证明其测量新鲜，重发旧数据也不能续期。适配器当前拒绝 `use_sim_time=true`，不会自动估算跨机器时钟偏移。

Odometry 和 VisionHealth 必须描述同一测量，stamp 相差 1 ns 也不配对。默认配对窗口为本机单调时间 0.05 s、每侧最多 32 个待配对项；不是按“最近时间”匹配。达到等待上限即过期，重复测量不能刷新水位。消费者另外检查源年龄、未来容差、接收 TTL 和序号推进；这些边界不能互相替代。

`time_sync_valid` 是上游声明，**不是适配器测出的时钟同步误差**。同目录的 `vision_state_validation.py::Validator` 是显式实例化才运行的离线验证器；它应用显式 ClockPolicy，不自动估计时钟偏移。实际 ROS 适配器接线使用 GuardedOdometryAdapter，不能把离线显式时钟偏移应用、连续性或协方差诊断说成 ROS 路径已经执行。

initialized、tracking、calibration_valid、米制尺度、世界对齐和 reset 均须有真实来源。重定位改变世界原点或 reset/source 变化时，旧计划不能继续沿用。跟踪丢失、无效字段、来源冲突等撤销事件与随后健康消息之间有专门水位语义。当前 Inbox 不持有所有 Controller 配置；例如 `quality` 在 `[0,1]` 内仍可能低于 Controller 的 `min_quality`。相关 API 历史候选及生产调度限制见阶段报告，不能把它误写为“任意坏帧必然在任意接线中锁存”。

## 质量与估计不确定性

quality 是双方约定的 `[0,1]` 评分，不自动代表概率、误差米数或协方差。VisionHealth 不能因为 Topic 有数据就一律填 `tracking=true`、`quality=1`。

协方差是 6×6 行主序矩阵；位置/线速度和旋转/角速度有各自单位。例如位置方差 0.04 m² 对应标准差 0.2 m。跨坐标、安装外参和杆臂的协方差需要完整变换；**当前适配器输出 NaN 且不设置两个协方差有效位**。本轮未伪造零方差，也未实现这一变换。未来如要消费协方差，需要明确误差定义、相关项及雅可比并另行验证。

## Topic、QoS 与 DDS

| 通路 | 当前默认/配置入口 | 当前接收或发布 QoS |
|---|---|---|
| 原始 Odometry 输入 | 适配器 `input_topic` 必填 | BEST_EFFORT；`odometry_reliable` 可配 |
| 原始 VisionHealth 输入 | 适配器 `health_topic` 必填 | RELIABLE；`health_reliable` 可配 |
| VisionState 输出 | `/fc01/vision/state`；适配器 `output_topic` | BEST_EFFORT；`output_reliable` 可配 |
| 生产 VisionState 输入 | `visual_state_topic` | BEST_EFFORT；`visual_state_reliable` 可配 |
| 生产 NavigationCommand 输入 | `/fc01/navigation/command`；`visual_command_topic` | RELIABLE；`visual_command_reliable` 可配 |
| 只读 VelocityCommand 输入 | `/fc01/vision/velocity_command`；诊断器 `topic` | 默认 BEST_EFFORT；`reliable` 可配 |

以上接口使用 VOLATILE。适配器默认队列深度 5、允许 1–1000；生产视觉输入允许 1–20，默认 5。这里列的是各端当前合同，没有将两个范围统一或修改。

RELIABLE 订阅不能匹配只提供 BEST_EFFORT 的发布者；BEST_EFFORT 订阅可接可靠发布者。QoS 不兼容时，ROS 图上“能看到发布者”仍可能收不到消息。发布者数量检查既不能证明 QoS 匹配，也不能证明来源可信；恢复为一个发布者不能复活旧候选。TTL 不因可靠传输而取消。

当前五种消息仍属于同一个 `jikl_fc01_system` 包。CMake 生成消息并链接本包 C++ typesupport，安装两个 Python 入口及两个相邻模块；package.xml 声明 Humble 所需运行依赖。新生成物检查器显式核验 ament、Python 类和类型支持都来自指定的新安装前缀，避免误用旧 overlay。

每个身份字符串的应用预算是 256 个 UTF-8 字节，不是 256 个汉字。此预算限制应用后续复制、缓存和日志路径，**不约束 DDS 在消息进入应用之前的分配**；不能据此宣称已具备传输层内存硬限。

## 与视觉同学仍须落实的输入

- 实际 Odometry、VisionHealth Topic、发布频率、QoS、相同 stamp 的生成方式与单位。
- 米制尺度、坐标轴、世界原点、世界对齐和相机到机体外参的测量记录及版本身份。
- 上游测量时钟域、跨机同步方法、延迟分布和重启后的 source/session/sequence/reset 规则。
- 角速度实际来源，跟踪丢失/重定位状态，quality 的定义和阈值依据；需要协方差时给出其定义与有效性。
- 带真实时序的录包、标定资料、丢帧和遮挡场景；Orange Pi Zero 3W 无动力 ARM64 构建、负载和内存测量。

本轮已做 x86_64 Humble 新构建、安装身份和合成通信验证；这些结果不替代上述材料。启用视觉、改变安全撤销语义和把 VelocityCommand 接入实际控制仍属于后续独立任务。
