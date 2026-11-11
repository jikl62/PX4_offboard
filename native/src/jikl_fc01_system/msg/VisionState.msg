# FC01 内部实际状态合同 V1；这不是相机厂商接口，也不是目标指令。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
uint16 POSITION=1 # valid_fields 的位置有效位；对应 NED 米制位置。
uint16 VELOCITY=2 # valid_fields 的速度有效位；对应 NED 实际速度。
uint16 ATTITUDE=4 # valid_fields 的姿态有效位；对应 FRD 到内部 NED 的旋转。
uint16 ANGULAR_VELOCITY=8 # valid_fields 的机体系角速度有效位。
uint16 POSE_COVARIANCE=16 # valid_fields 的位置/姿态协方差有效位；未经完整变换不得设置。
uint16 TWIST_COVARIANCE=32 # valid_fields 的速度/角速度协方差有效位；未经完整变换不得设置。
uint16 QUALITY=64 # valid_fields 的质量有效位；缺失质量不能填成满分。
uint64 sequence # 本发送会话内推进的非零序号；重复帧不能延长有效期。
builtin_interfaces/Time stamp # 双方明确约定时钟域中的测量时间，不能用回调接收时间替代。
uint32 reset_counter # 估计或任务周期标识；变化必须按对应通道的重置规则处理。
string frame_id # 内部世界系 fc01_vision_ned；x 北向参考、y 东向参考、z 向下。
string body_frame_id # 机体坐标系标识；内部使用 base_link_frd（前、右、下）。
string source_id # 明确的发送会话来源；话题有数据不代表来源已获授权。
string angular_velocity_source # 角速度测量来源；声明角速度有效时不能为空。
bool initialized # 估计器已完成初始化；默认 false，不从消息到达推断。
bool tracking # 估计器当前跟踪正常；不等同于 ROS 链路仍连通。
bool calibration_valid # 当前使用的标定已确认有效；缺少确认时拒绝有效反馈。
uint16 valid_fields # 各数值的有效位掩码；默认 0，默认数组中的零不代表真实测量。
float64[3] position # 实际位置 [x,y,z]，单位 m，内部 NED 系。
float64[3] velocity # 实际速度 [vx,vy,vz]，单位 m/s，内部 NED 系。
float64[4] attitude # Hamilton [w,x,y,z]，将机体 FRD 旋转到内部 NED；不是 ROS xyzw 顺序。
float64[3] angular_velocity # 实际角速度 [ωx,ωy,ωz]，单位 rad/s，机体 FRD 系。
float64[36] pose_covariance # 行主序 6×6 矩阵，顺序为位置 xyz、固定轴角度 xyz；另有有效位。
float64[36] twist_covariance # 行主序 6×6 矩阵，顺序为线速度 xyz、角速度 xyz；另有有效位。
float64 quality # 已确认归一化的质量 [0,1]；仅在 QUALITY 位设置时有效。
