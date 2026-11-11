# 视觉身份应用层资源合同 V1 补充

ROS 类型和字段不变，新增准入条件：每个身份字段最多 **256 UTF-8 字节**，不含内嵌 NUL，不含非法 UTF-8/Unicode surrogate。不是 256 个中文字符。空字段是否允许仍按原字段合同；必填 source/frame/calibration 继续要求非空，资源门不补默认身份。

| 路径 | 覆盖字段 | 拒绝行为 |
|---|---|---|
| Odometry | header.frame_id、child_frame_id | Guarded 入口先输出已超时撤销、清两侧半包，再输出新的无效状态；低层 PairBuffer 独立调用会抛明确 ValueError，不缓存坏半包。 |
| VisionHealth / adapter 配置 | source_id、frame_id、body_frame_id、angular_velocity_source、extrinsics_id、alignment_id；配置中相应 input_* 身份 | 配置拒绝启动；消息拒绝不回显异常原文。只有新的完整配对恢复，不截断后接受。 |
| VisionState | source_id、frame_id、body_frame_id、angular_velocity_source | ROS 字段首次复制前写本机无效标记，Inbox 撤销；保留旧有界身份、reset 和序号高水位，旧序号不能因异常包重新续鲜。 |
| NavigationCommand / VelocityCommand | source_id、frame_id | 与状态相同的预算；两类命令语义仍分开。VelocityCommand 诊断始终 control_authority=false。 |
| 离线状态元信息 | relocalization_id | 配置/元数据校验拒绝超限，不增加可信 epoch。 |
| JSON 日志 | 不可信身份的显示副本 | 最多 256 UTF-8 字节，按字符边界裁切；显示文本不能用于来源认证。固定拒绝原因不拼接巨大原文。 |

C++ 的 vision_resource_budget.hpp 与 Python 的 vision_resource_budget.py 共享字节限制，并有跨语言边界测试。独立审查额外比较了 96,488 个字节序列的 UTF-8 判定，未出现差异。合法 256 字节与超限 257 字节、1 MiB、NUL、非法 Unicode 都有回归。

边界依据是复用已有速度预览 source_id 的 256 字节合同，同时为 UUID、包/坐标命名、会话/标定版本留出余量；不是传感器物理阈值。视觉同学应同步这一新合同，避免将路径、整段 JSON 或图像元数据放进身份字段。需要更长身份时，应重新评估总缓存/日志预算并明确版本，不能单方面截断身份。

当前生产配对缓存上限仍为 max_pending，Inbox 仍只存最新状态/命令，可信 epoch/历史仍各自有界；每身份上界不能直接当作整个进程内存上界。输入消息首先由 ROS/DDS 反序列化，其分配可能早于应用门；本轮没有修改 wire schema 为 bounded string，没有承诺拒绝巨型网络报文前零分配。吞吐、QoS 队列和 OS/RMW 的内存限额需要 ARM64 与实际视觉录包另测。
