# 冲突与裁决

用户当前明确指令和 01/03/04 合同决定本次授权范围；00 的事实优先级用于区分真机事实、实际源码、v1.16 接口和 C 推论。

| 材料差异 | 高优先级证据/要求 | 裁决 |
| --- | --- | --- |
| C 提议每周期 current-yaw；03 要进入准备时捕获局部 reference | 01/03 正式合同 | 固定捕获 reference，不做可靠北向控制；disarmed quat reset 可 rebase |
| C 的早期方案允许 RC/QGC Arm；01/03 第一阶段任何 armed 都拒绝 | 01/03 与用户当前绝对要求 | 第一阶段 armed → SAFETY，停止 healthy output，不续发 zero-thrust heartbeat |
| A/B 旧报告称 ACK/完整 build 未验证，B 对 external=false 尚存推论 | 02 后续实机日志与明确 PASS | 继承真机已验证 ACK 规则；不重复猜测、不修改 matcher |
| C 要调查 PID/ESC/2/4 电机与历史 ULog | 当前任务书明确排除 | 本交付只实现软件控制与 gate；这些不作为本次编码工作 |
| C 引用若干 main/v1.14/v1.15 接口说明 | 02 的实际 v1.16 消息字段与固定 commit；现有 r2 static assertions | 实际新增接口只按 v1.16/MESSAGE_VERSION=0；不搬 main 字段 |
| 基线历史 `.before_message_version_fix` 对 Mode 要求 MESSAGE_VERSION，实际 r2 节点已去掉 | 真实编译的基线 .cpp + 02 OffboardControlMode 字段 | 保持实际 r2 约束，新增 Attitude 两个版本断言；历史快照不编译 |
| 基线 install 首次安装创建 current；04 要不得自动切 current | 用户当前交付要求/04 | 首次和后续均只构建 release；显式 update.sh 激活 |
| 05 的“参数全部 YAML”与 C 的 suggested gains | 不得猜 gains/hover thrust，必须通过用户配置和 validation | 安全准备参数明确 YAML；飞行参数 -1 sentinel，确认 false |
| flight/sim 保留原路径，而新 master flight gate 文本很严格 | 03 profile 条款 + 实际基线 ACK 单测依赖原导航门 | 新三门作用于 attitude_altitude；旧 nav 仍由 flight_enabled 和原状态机约束，第一阶段禁止选 sim |

没有证据证明厂商 Commander/bridge 与 upstream 二进制逐行/逐指令相同，结论 UNKNOWN。0.4 完整 ROS/Humble 编译、实际 DDS 接口与 Offboard readiness、Arm/Disarm 真机路径、ARM64 原生 build 均 UNKNOWN。任何未确认参数均没有从聊天摘要补入。
