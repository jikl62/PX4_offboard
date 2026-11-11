# 一条原始 Odometry 测量的显式健康信息，必须按完全相同的测量时间戳配对。所有确认默认 false。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
builtin_interfaces/Time stamp # 双方明确约定时钟域中的测量时间，不能用回调接收时间替代。
uint64 sequence # 本发送会话内推进的非零序号；重复帧不能延长有效期。
string source_id # 明确的发送会话来源；话题有数据不代表来源已获授权。
string frame_id # 必须与这条原始 Odometry 的 header.frame_id 完全一致。
string body_frame_id # 必须与这条原始 Odometry 的 child_frame_id 完全一致。
uint32 reset_counter # 原始估计器周期标识；只有 reset_valid=true 时才算已明确提供。
bool reset_valid # 发送方明确保证 reset_counter 有效；不能把默认零当成已知周期。
bool initialized # 估计器已完成初始化；默认 false，不从消息到达推断。
bool tracking # 估计器当前跟踪正常；不等同于 ROS 链路仍连通。
bool calibration_valid # 当前使用的标定已确认有效；缺少确认时拒绝有效反馈。
bool frame_alignment_valid # 视觉世界坐标与内部导航方向/原点的对齐已确认。
bool time_sync_valid # 测量时间与约定 ROS/system 时钟的对应关系已确认。
bool extrinsics_valid # 传感器到机体的外参及杆臂已确认，不能猜测安装方向。
bool metric_scale_valid # 位置和速度已具有真实米制尺度；单目任意尺度不能直接用于控制。
bool position_valid # 本次原始位置测量有效；无效时不允许补零冒充位置。
bool velocity_valid # 本次实际速度测量有效；这不是上层要求的目标速度。
bool attitude_valid # 本次实际姿态测量有效；原始 ROS 四元数由适配器转换。
bool angular_velocity_valid # 本次角速度测量有效；非零杆臂速度转换依赖它。
string angular_velocity_source # 角速度测量来源；声明角速度有效时不能为空。
bool quality_valid # 质量数值的定义和量纲已由发送方确认。
float64 quality # 内部归一化质量 [0,1]；仅在 quality_valid=true 时有效。
string extrinsics_id # 绑定明确外参版本；相同标识不应悄悄替换外参。
string alignment_id # 绑定明确坐标对齐版本；重定位/更换地图需按协议声明。
