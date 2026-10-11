# PX4 v1.16 官方资料核对

核对日期：2026-10-05。仅使用 v1.16 官方资料；没有以 main 替换指定版本。本文说明设计依据，不证明现场行为，也不提供 PX4 参数修改操作。

| 官方页面 | 本次核对内容 |
| --- | --- |
| [Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard) | 自主模式可以不依赖 RC；Offboard 外部控制仍受其原生信号和失联机制约束 |
| [Parameter Reference](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference) | COM_RC_IN_MODE=4 禁用 stick；COM_RC_OVERRIDE 的 bit0 用于 auto、bit1 用于 Offboard，阈值另由 COM_RC_STICK_OV 控制；COM_RCL_EXCEPT 的 bit2 涉及 Offboard RC-loss exception；COM_OF_LOSS_T 与 COM_OBL_RC_ACT 分别涉及 Offboard-loss 延时及动作。动作名称不能证明无 XY 定位时的物理安全 |
| [InputRc](https://docs.px4.io/v1.16/en/msg_docs/InputRc) | timestamp_last_signal、channel_count、input_source、rc_lost、rc_failsafe 可供只读观测。实现排除 unknown/MAVLink 来源，不把有效数据当成已校准证明 |
| [ManualControlSwitches](https://docs.px4.io/v1.16/en/msg_docs/ManualControlSwitches) | timestamp_sample；NONE=0、ON=1、MIDDLE=2、OFF=3；uint32 switch_changes 报告开关变化数。实现同时观察位置与计数，以保守识别遗漏的中间状态 |
| [VehicleStatus](https://docs.px4.io/v1.16/en/msg_docs/VehicleStatus) | TERMINATION nav_state=13 和 failsafe_and_user_took_over 作为释放事实；实际映射仍需现场验证 |
| [FailsafeFlags](https://docs.px4.io/v1.16/en/msg_docs/FailsafeFlags) | offboard_control_signal_lost、fd_critical_failure 等只读失效事实 |

指定 px4_msgs pin `392e831c1f659429ca83902e66820d7094591410` 未改。新消息的精确 pin schema、真实 Humble 编译、FC01 firmware 是否向 DDS 发布 input_rc/manual_control_switches，均未获得实际验证，保持 UNKNOWN。类型 fixture 仅证明当前适配代码的语法/API 预期；没有为新消息臆造 MESSAGE_VERSION。缺失 RC topic 会拒绝 RC 路线 readiness，自主路线仍有其独立 false-default confirmation。
