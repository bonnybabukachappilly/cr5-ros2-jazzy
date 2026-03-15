# Joint space state
float64[6] joint_positions
float64[6] joint_speeds
float64[6] joint_torques

# Cartesian space state
float64[6] tcp_pose

# Robot status
bool enabled
int32 mode
string error_code

# Timestamp of last feedback packet
float64 last_update