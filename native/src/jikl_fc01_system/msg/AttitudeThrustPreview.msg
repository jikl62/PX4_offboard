# 阅读顺序：accepted/reason说明输入检查 → 序号/reset追溯来源 → 目标量说明计算 → sink观测说明边界。
# 这是观察/离线预览合同；不是向PX4发送的VehicleAttitudeSetpoint，也不是电机驱动接口。
# active或accepted为true仅说明对应预览结果；不能替代RC控制权、飞行动作门和真正发布证据。
# attitude/thrust_body是目标，不能当实测姿态/实际推力；推力为归一化比例，-0.5不等于-0.5N。
# velocity_target/acceleration_target在世界NED，target_height却是相对地面向上为正，注意符号区别。
# 例如velocity_target.z=-0.1表示向上速度目标0.1m/s；target_height=1.5表示地面参考上方1.5m。
# state_sequence和command_sequence来自两个独立序列，数字相等不代表同步；stamp也不证明执行。
# 两个sink_allowed只保留相应检查/提交的观察结果，不能单凭本消息证明PX4采用或飞机已离地。
# 离线观测消息；不是执行器命令，也不是 PX4 输入。
uint16 schema_version 1 # 协议版本；默认 1，接收方必须拒绝不兼容的版本。
builtin_interfaces/Time stamp # 该预览的时间记录；不能单凭它认定飞控已经执行。
bool accepted # 预览输入是否通过对应检查；不表示获得飞行控制权限。
bool active # 预览是否处于运动计算状态；不是实际电机状态。
bool request_land # 预览提出的降落要求；需要原权限和控制链执行。
string reason # 预览接受、拒绝或退出的原因字符串。
uint8 mode # 对应内部导航模式；含义与 NavigationCommand 模式一致。
uint64 state_sequence # 生成预览时所使用的实际状态序号。
uint64 command_sequence # 生成预览时所使用的目标命令序号，与状态序号独立。
uint32 reset_counter # 估计或任务周期标识；变化必须按对应通道的重置规则处理。
string frame_id # 世界坐标系标识；具体约定必须与接收配置一致。
string body_frame_id # 机体坐标系标识；内部使用 base_link_frd（前、右、下）。
float64[4] attitude # Hamilton [w,x,y,z]，机体 FRD 到内部 NED 的姿态目标。
float64[3] thrust_body # 机体 FRD 的归一化推力向量；向上推力的 z 分量为负。
float64 target_height # 相对当前起飞地面参考的目标高度，单位 m，正值向上。
float64 yaw # 内部 NED 航向目标，单位 rad。
float64[3] velocity_target # 世界 NED 速度目标 [vx,vy,vz]，单位 m/s。
float64[3] acceleration_target # 世界 NED 加速度目标 [ax,ay,az]，单位 m/s²。
bool attitude_sink_allowed # 姿态提交检查的观测结果；不等同于实际 PX4 接收证据。
bool thrust_sink_allowed # 推力提交检查的观测结果；不等同于实际动力执行证据。
