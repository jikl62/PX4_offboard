# Offline observation only; never an actuator or PX4 input message.
uint16 schema_version 1
builtin_interfaces/Time stamp
bool accepted
bool active
bool request_land
string reason
uint8 mode
uint64 state_sequence
uint64 command_sequence
uint32 reset_counter
string frame_id
string body_frame_id
# Hamilton w,x,y,z: body FRD -> fc01_vision_ned.
float64[4] attitude
# Dimensionless normalized body FRD thrust; upward thrust has negative z.
float64[3] thrust_body
float64 target_height
float64 yaw
float64[3] velocity_target
float64[3] acceleration_target
bool attitude_sink_allowed
bool thrust_sink_allowed
