# Upper-level desired velocity only; this message grants no flight authority.
uint16 schema_version 1
uint64 sequence
# Command computation time in the agreed ROS/system time domain, not receipt time.
builtin_interfaces/Time stamp
# Sender reset epoch; diagnostic ingress records it independently of VisionState.
uint32 reset_counter
# Publisher session identity. A restarted producer must identify its new session.
string source_id
# Must be explicitly set to "body_frd": body x forward, y right, z down.
string frame_id
# false withdraws this request; it must not renew an earlier nonzero request.
bool valid
# Desired body FRD velocity in m/s. These are targets, not measured velocities.
float64 vx
float64 vy
float64 vz
