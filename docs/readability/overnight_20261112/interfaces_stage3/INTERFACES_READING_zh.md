# 从服务入口读到视觉状态和资源释放

这份路线帮助初学者在 VS Code 中定位本阶段源码，不替代函数旁的具体中文合同。用 `Ctrl+Shift+F` 搜索以下函数/类型名即可跟到真实实现；不要按文件的历史版本名猜运行路径。

## 先分清“允许配置”“当前授权”和“已经执行”

`navigation_service.py::_run_navigation_service` 生成配置并交给监督器。`checked_runtime_profile` 是“以后每次启动时再调用”的函数对象，当前不执行飞行控制。每次调用仍先 `check_controller()`，再 `load_profile(...)`，加载失败不返回旧配置。`args.mode` 是闭包中对象的实时属性，`ENTRY` 是实时查询的全局值，不在定义工厂时缓存。

YAML 的 `flight_actions_enabled` 只是其中一道配置门。原生状态机还要检查实际反馈、阶段、RC 边沿、Kill、控制权等；计划通过后，最终发布前仍会重新检查。`arm_sent_` 等规划状态的含义必须结合所属 Core 阅读，不能仅凭名字认定真实 ROS 命令已经提交。`ground_link.hpp` 中的 `jikl_fc01_ground_link::Core` 是独立地面诊断链，不能把它的 ACK 处理当作正式 RC 导航解锁流程。

代码里四件事不同：算出目标、调用 sink、目标与心跳都成功提交、后续飞控反馈证明采用。目标/心跳两次发布不是原子操作；本轮没有移动这条时序，也没有把中途失败标成完整成功。具体控制路线继续读上一阶段 `CPP_READING_zh.md`。

## 视觉实际状态与制导目标分开读

| 对象 | 表达什么 | 单位/方向 | 当前作用 |
|---|---|---|---|
| 原始 `nav_msgs/Odometry` | 估计器认为飞机/传感器现在在哪里、如何运动 | 必须明确 frame、body、时钟、外参 | Python adapter 根据显式配置转换，不能仅改 frame 名称。 |
| `VisionState.position/velocity/attitude` | 转换后的自身实际状态 | 位置 m；速度 NED m/s；姿态 body FRD 到 NED，Hamilton w/x/y/z | 供验证/状态路径使用；不能填成期望运动。 |
| `VelocityCommand.vx/vy/vz` | 制导希望飞机如何运动 | 机体 FRD m/s：前、右、下 | 独立只读接收、诊断和离线预览；没有飞行权限。 |
| `NavigationCommand` | 既有模式、位置、航向或归一化杆量目标 | 各字段按旧协议 | 与 VelocityCommand 是不同消息，不能互换。 |
| `AttitudeThrustPreview` | 离线计算得到的姿态/推力候选与诊断 | 字段约定见 MSG | 预览通过不等于 Runtime 发布授权或 PX4 采用。 |

例如：飞机朝东且水平，`VelocityCommand(vx=0.3, vy=0, vz=0)` 希望沿机头向前，即 NED 的东向约 0.3 m/s。如果此刻实际 `VisionState.velocity=[0,0,0]`，预览才计算目标与实际的速度误差；把目标速度填进实际速度会把误差伪造成零。飞机倾斜时必须按完整实测四元数旋转，不能只按 yaw 转换，也不能把 FRD 的 vz 直接当作世界垂向速度。

ROS 常见四元数字段顺序 x/y/z/w 与内部数组 w/x/y/z 不同；必须看明确映射。`vision_ros_adapter.hpp` 只把本工程生成消息的字段复制到纯 C++ 对象，不是 ENU/FLU 坐标变换器。原始 Odometry 的变换在 Python adapter，来源、标定与健康配对仍要独立验证。

## 从消息字段追到拒绝条件

先读五份 MSG 的中文合同，再搜索 `state_from_message`、`command_from_message`、`fc01::vision::Inbox`、`fc01::velocity_input::Session` 和 `velocity_core.hpp`。

1. `valid_fields` 是位掩码。只有声明有效的数据才复制；全零数组本身不说明测量有效。`1 | 2 | 4 | 64 = 71` 声明位置、速度、姿态和质量，不声明角速度或协方差。
2. `sequence`、`source_id`、`reset_counter` 和时间戳用于连续性与防重放；正常递增序号不等于重置。拒绝包不能通过覆盖健康标志抹掉已经发生的异常，也不能随意推进可信水位。
3. 视觉状态估计器 reset 与目标速度发送方 reset 是各自的周期，数字碰巧相同不证明属于同一周期。重定位/来源变化须按具体协议处理。
4. 所有身份最多 256 UTF-8 **字节**，不是 256 个汉字；空串是否允许由具体字段协议判断。无效身份直接拒绝，不能截断后当作同一个有效来源。
5. 复制/缓存前的应用预算不限制 DDS 已发生的反序列化分配；注释没有承诺进程总内存封顶。

时间至少有源测量时刻、本机 ROS/system 当前时刻、本机单调接收时刻和本机单调当前时刻。源戳为 100.0 s、同域当前为 100.1 s，源年龄是 0.1 s；本机接收为 250.70 s、当前为 250.75 s，接收年龄是 0.05 s。不能用 `250.75 - 100.0` 计算延迟，因为两种时钟可能有不同 epoch。重复包刷新收到消息的时刻，也不能刷新真实测量寿命。`max_measurement_delay` 与到达后缓存 TTL 不是同一预算。

`source_diagnostics.hpp` 提供有符号年龄和首个失败原因，只用于解释保护。负年龄、缺时间戳、raw 不推进、接收过期不能都填成零。RMW 接收戳只有时钟域已知才有可解释的差值；这个差值本身不证明队列延迟，也不能修正消息或覆盖 TTL。

## C++ 语法和资源所有权怎么读

| 写法 | 本工程中的含义与阅读要点 |
|---|---|
| `const T& value` | 借用一个对象，本函数不通过该引用修改它；不自动延长任意外部资源寿命。同步回调借用参数不得逃逸保存。 |
| `std::string_view` | 借用字符串字节；预算函数只扫描，不保存视图，不复制字符串。 |
| `std::optional<T>` | 明确区分“没有值”和“有一个零值”；访问前检查是否存在。 |
| `std::move(value)` | 允许接收方移动资源，不是立即释放；队列入队成功后才能提交字节计数。 |
| `std::unique_ptr<T>` | 当前对象唯一持有该资源，析构时释放；raw fd 仍需沿具体失败分支人工 close。 |
| `lock_guard` / `unique_lock` | 局部对象通过生命周期持有锁；条件变量等待可能暂时释放再取得锁，不能把异步日志当无锁硬实时系统。 |
| `template<class Message>` | 真实 ROS 生成类型与离线夹具复用同一字段转换；模板不自动验证字段语义或替你转换坐标系。 |
| `if (a && b)` / 早返回 | 短路与首拒绝顺序有实际含义；不能为了排版把有副作用的判断重新排序。 |

`socket_lease.hpp` 用 inode 与路径双重身份判断清理对象，不因路径名字相同就 unlink 别人的 socket。`unix_server.hpp` 中 raw fd 与租约对象的责任不同；捕获范围内的 std::exception 才走相应断连逻辑，不承诺任意异常都已经处理。

`setpoint_ownership.hpp` 检查图上的竞争发布者；“此刻未观察到竞争”不是永久租约，也不是飞控已经采用。`authority_telemetry.hpp` 解释 PX4 选择后的权限来源；不能拿原始遥控通道输入替代真实选中来源，遥测 Kill 与任务锁存也不是同一个布尔量。

## Python 的正常退出和异常退出

`velocity_command_diagnostic.py` 是只读工具。main 先取得完整基类 Node，再执行 `configure_diagnostic`，让配置失败也落在 main 的资源所有权范围。Receiver 只判断诊断候选；全路径 `control_authority=false`，没有飞控 publisher。

参数、订阅、timer、首快照任一步失败，原异常都继续可见；清理列表仍逐项尝试。退出写盘失败也会报错。`DiagnosticWriter.record()` 返回 False 只说明当前传入事件没写出，本轮仍可能先刷新旧的待写异常。`snapshot()` 的原子替换防止读到半份 JSON，不等于 fsync 或断电持久性保证。

`navigation_restore.py` 是外部事务：改服务/配置前后用水位和补偿记录区分“没有尝试”和“可能部分完成”。`rollback(stop_current=True)` 才请求/等待原控制器停止；False 路径检查其已经停止。恢复备份不自动启动旧飞行任务。

`standby.py` 区分自建 Agent 与复用 Agent，退出时保留缺少安全退出证据的自建 Agent，不拥有随意停止复用者的权利。monitor 的已处理异常与 supervise 的 finally 是不同层次；顺序调用清理尚不保证每个清理动作自己抛错后仍能释放其余所有资源。这一边界已明确，后续须先故障注入再考虑行为修复。

最后读 [阶段报告](README_zh.md) 的真实测试结果和未完成项。源码可读、主机差分与真实 ROS 合成通信分别解决不同问题，均不能替代 Orange Pi、PX4 日志和现场硬件证据。
