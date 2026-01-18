# Landmark verification result
std_msgs/Header header

string label
uint32 landmark_id

# Expected vs observed positions
geometry_msgs/Point expected_position
geometry_msgs/Point observed_position

# Error
geometry_msgs/Vector3 error_xyz
float64 error_distance

# Status:  OK, DEGRADED, CRITICAL, NOT_FOUND
string status
float32 confidence
