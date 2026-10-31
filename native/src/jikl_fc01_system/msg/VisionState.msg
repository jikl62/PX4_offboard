# Temporary FC01 internal contract V1; this is not a vendor camera interface.
uint16 schema_version 1
uint16 POSITION=1
uint16 VELOCITY=2
uint16 ATTITUDE=4
uint16 ANGULAR_VELOCITY=8
uint16 POSE_COVARIANCE=16
uint16 TWIST_COVARIANCE=32
uint16 QUALITY=64
uint64 sequence
# Measurement time in the explicitly agreed ROS/system domain; not callback time.
builtin_interfaces/Time stamp
uint32 reset_counter
string frame_id
string body_frame_id
string source_id
string angular_velocity_source
bool initialized
bool tracking
bool calibration_valid
# Default mask 0 means every default numeric payload is UNKNOWN.
uint16 valid_fields
float64[3] position
float64[3] velocity
# Hamilton w,x,y,z, rotating body FRD into fc01_vision_ned. Not ROS xyzw.
float64[4] attitude
# Body FRD, rad/s. Valid only with ANGULAR_VELOCITY and nonempty source.
float64[3] angular_velocity
# Row-major 6x6, linear xyz then fixed-axis angular xyz. Separate validity bits.
float64[36] pose_covariance
float64[36] twist_covariance
# Explicit normalized [0,1] internal quality; unknown unless QUALITY is set.
float64 quality
