# 内部运动请求；不能授权 ARM、Offboard 或任何控制租约。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
uint8 DISABLED=0 # 撤销运动输入；已有视觉运动按原退出规则处理。
uint8 HOLD=1 # 进入时捕获当前位置；需要可信实际状态反馈。
uint8 CRUISE=2 # 前往明确的内部 NED 绝对点，不是相对位移。
uint8 FOLLOW=3 # 接收四路归一化运动请求，经视觉姿态转换到世界坐标。
uint8 LAND=4 # 请求原控制器受控降落；本消息不能授权解锁或 Offboard。
uint64 sequence # 本发送会话内推进的非零序号；重复帧不能延长有效期。
builtin_interfaces/Time stamp # 双方明确约定时钟域中的测量时间，不能用回调接收时间替代。
uint32 reset_counter # 估计或任务周期标识；变化必须按对应通道的重置规则处理。
string source_id # 明确的发送会话来源；话题有数据不代表来源已获授权。
string frame_id # 世界坐标系标识；具体约定必须与接收配置一致。
uint8 mode # 上述运动模式；本字段不携带飞行控制权限。
bool valid # 本帧请求是否有效；false 不能续期上一条运动目标。
bool position_valid # 绝对点目标是否明确提供；Cruise 使用前必须为 true。
bool yaw_valid # 绝对航向是否明确提供；无效时沿用控制器既有航向规则。
bool sticks_valid # 四路请求是否明确提供；仅 Follow 模式消费。
float64[3] position # 绝对点 [x,y,z]，单位 m，内部 fc01_vision_ned。
float64 yaw # 内部 NED 绝对航向，单位 rad；需要 yaw_valid=true。
float64[4] sticks # [roll,pitch,yaw,throttle]，各为 [-1,1]；正值为右、前、右转、上升。
