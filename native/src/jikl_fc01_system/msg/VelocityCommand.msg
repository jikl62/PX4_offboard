# 上层目标速度接口；当前独立接收和诊断，不授予飞行权限。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
uint64 sequence # 本发送会话内推进的非零序号；重复帧不能延长有效期。
builtin_interfaces/Time stamp # 目标速度的计算时刻，使用约定的 ROS/system 时钟；不是接收时刻。
uint32 reset_counter # 速度发送方的重置周期；独立于 VisionState 的估计器周期记录。
string source_id # 明确的发送会话来源；话题有数据不代表来源已获授权。
string frame_id # 必须显式填写 body_frd：x 前、y 右、z 下；字符串默认仍为空。
bool valid # false 撤销本次请求；不能借此续期此前的非零目标速度。
float64 vx # 机体前向目标速度，单位 m/s；这是目标，不是实际速度反馈。
float64 vy # 机体右向目标速度，单位 m/s；必须是有限数值。
float64 vz # 机体向下目标速度，单位 m/s；负值表示向上。
