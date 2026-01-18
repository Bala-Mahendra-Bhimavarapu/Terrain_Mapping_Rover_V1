# Single waypoint
std_msgs/Header header

uint32 waypoint_id
geometry_msgs/PoseStamped pose

# Optional metadata
string label
float32 tolerance_position   # meters
float32 tolerance_orientation  # radians

# Recording info
builtin_interfaces/Time recorded_at
string recording_method  # TIME, DISTANCE, EVENT