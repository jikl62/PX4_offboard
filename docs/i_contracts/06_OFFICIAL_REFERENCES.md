# PX4 v1.16 references relevant to authority modes

Use v1.16 only for version-sensitive behavior. Do not use `main` to override v1.16.

- Offboard Mode: https://docs.px4.io/v1.16/en/flight_modes/offboard
- Parameter Reference: https://docs.px4.io/v1.16/en/advanced_config/parameter_reference
- RC systems: https://docs.px4.io/v1.16/en/getting_started/rc_transmitter_receiver
- Radio setup: https://docs.px4.io/v1.16/en/config/radio
- Safety/failsafe: https://docs.px4.io/v1.16/en/config/safety
- Failsafe simulation: https://docs.px4.io/v1.16/en/simulation/failsafes
- ROS 2 guide: https://docs.px4.io/v1.16/en/ros2/user_guide

Known official facts to re-verify before implementation claims:
- PX4 does not require RC for autonomous flight modes.
- COM_RC_IN_MODE=4 ignores stick input.
- COM_RC_OVERRIDE bit 1 enables stick override in Offboard; threshold is COM_RC_STICK_OV.
- COM_RCL_EXCEPT bit 2 exempts Offboard from RC-loss failsafe.
- COM_OF_LOSS_T controls Offboard-loss timeout; COM_OBL_RC_ACT controls requested loss action.
