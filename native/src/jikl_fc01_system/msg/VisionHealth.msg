# 本消息为nav_msgs/Odometry补齐“数值存在也无法推断”的质量与标定承诺，本身不提供运动目标。
# 发布者必须针对同一条Odometry测量使用完全相同stamp，并明确source_id和原始父/子坐标名。
# 例如100s+20ns不能与100s+21ns配成同一测量；接收先后顺序相邻也不代表同一帧。
# position_valid等为本次测量确认；frame/time/extrinsics/metric等则确认转换前提，各自不可互代。
# 例如tracking=true不能证明米制尺度或外参正确，time_sync_valid=true也不能让旧数据重新变新。
# reset_valid明确声明周期已知；默认reset_counter=0不自动等于“估计器从未重置”。
# 非零安装杆臂的速度转换需要角速度和其来源；没有角速度不能假设杆臂造成的速度为0。
# extrinsics_id/alignment_id绑定已约定版本；更换地图或重定位需要按周期/撤销规则处理。
# 应用层对身份文本采用每字段256个UTF-8字节预算；质量/变换适配器仍逐项审核并配对。
# 配对通过只生成内部实际状态，仍不授权程序解锁、Offboard或动力输出。
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
