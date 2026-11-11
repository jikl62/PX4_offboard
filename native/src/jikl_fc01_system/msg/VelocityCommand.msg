# 固定类型：jikl_fc01_system/msg/VelocityCommand；约定订阅话题/fc01/vision/velocity_command。
# 本消息是制导模块的目标速度；VisionState.velocity才是视觉估计的实际速度，二者用途不同。
# vx/vy/vz在body_frd：前/右/下，单位m/s；不能填NED世界系，也不能填归一化遥控杆量。
# 例：vx=0.3表示希望沿机头前进0.3m/s；vz=-0.1表示希望沿机体上方运动0.1m/s。
# 飞机倾斜时机体向前会包含世界垂向分量，离线预览按完整实测四元数旋转后才与NED反馈相减。
# 例：水平机头朝东时vx=0.3旋转成NED目标[0,0.3,0]；这不说明实际东向速度已达到0.3m/s。
# 本协议没有yaw目标字段，预览必须从外部显式接收可信航向参考，不能自行假定为0。
# source_id/frame_id逐项最多256个UTF-8字节，不截断来源文本后冒充同一身份。
# sequence/stamp在同一来源/周期内应推进，重复包不续期；valid=false或异常必须撤销旧资格。
# reset属于速度生产方，独立于VisionState估计器reset；不能因数字恰好相等就声称二者是同一周期。
# 当前接收和预览均不进入生产Runtime，不改变视觉开关、起飞目标、高度闭环或控制权限。
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
