# 本消息回答“旧视觉导航外环希望使用什么模式/位置/归一化杆量”，不是实测状态。
# 它不同于VelocityCommand的FRD米/秒速度合同；FOLLOW sticks是[-1,1]请求，不能直接当m/s。
# HOLD只在进入时锁当前位置，CRUISE给世界系绝对点，FOLLOW按当前视觉yaw转换水平杆量。
# LAND仍须命令来源/时间/有效性检查，只是可在缺米制视觉状态时请求原下降路径。
# reset_counter必须匹配VisionState估计周期；这与VelocityCommand独立生产方reset的约定不同。
# sequence是本发送会话水位、stamp是源生成时刻；回调接收时间由本机另记，不在本消息里伪造。
# 身份字段按每字段256个UTF-8字节预算检查；来源文字和单一发布者均不能单独构成控制权限。
# position/yaw/sticks各自的valid只表示提供了字段，最终还查有限性、模式所需项和飞行包络。
# 内部运动请求；不能授权 ARM、Offboard 或任何控制租约。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
uint8 DISABLED=0 # 撤销运动输入；已有视觉运动按原退出规则处理。
uint8 HOLD=1 # 进入时捕获当前位置；需要可信实际状态反馈。
uint8 CRUISE=2 # 前往明确的内部 NED 绝对点，不是相对位移。
uint8 FOLLOW=3 # 四路归一化运动请求；水平按视觉偏航转换到世界系，垂向独立处理。
uint8 LAND=4 # 请求原控制器受控降落；本消息不能授权解锁或 Offboard。
uint64 sequence # 本发送会话内推进的非零序号；重复帧不能延长有效期。
builtin_interfaces/Time stamp # 指令生成时刻，使用明确约定的时钟域；不能用接收时间替代。
uint32 reset_counter # 绑定实际状态的估计周期；生产控制器要求与 VisionState 的值相同。
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
