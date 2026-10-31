# Explicit metadata for ONE raw Odometry measurement, joined by exact stamp.
# All confirmations default false. A live topic does not imply tracking/quality.
uint16 schema_version 1
builtin_interfaces/Time stamp
uint64 sequence
string source_id
# Exact raw Odometry header.frame_id and child_frame_id.
string frame_id
string body_frame_id
uint32 reset_counter
bool reset_valid
bool initialized
bool tracking
bool calibration_valid
bool frame_alignment_valid
bool time_sync_valid
bool extrinsics_valid
bool metric_scale_valid
bool position_valid
bool velocity_valid
bool attitude_valid
bool angular_velocity_valid
string angular_velocity_source
bool quality_valid
# Internal normalized [0,1], only valid when quality_valid; vendor scale needs adapter.
float64 quality
# Bind to immutable, explicitly configured calibration/alignment identifiers.
string extrinsics_id
string alignment_id
