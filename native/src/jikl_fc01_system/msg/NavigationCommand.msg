# Motion request only. This message cannot authorize ARM, Offboard, or any lease.
uint16 schema_version 1
uint8 DISABLED=0
uint8 HOLD=1
uint8 CRUISE=2
uint8 FOLLOW=3
uint8 LAND=4
uint64 sequence
builtin_interfaces/Time stamp
uint32 reset_counter
string source_id
string frame_id
uint8 mode
bool valid
bool position_valid
bool yaw_valid
bool sticks_valid
# Absolute point in fc01_vision_ned, metres; Cruise only when position_valid.
float64[3] position
# Absolute NED yaw, radians, when yaw_valid.
float64 yaw
# roll/pitch/yaw/throttle in [-1,1], valid only for Follow with sticks_valid.
float64[4] sticks
