# 3D Landmark position
std_msgs/Header header

string label
uint32 landmark_id

# 3D position in map frame
geometry_msgs/Point position

# Covariance (xx, yy, zz, xy, xz, yz)
float64[6] covariance

# Detection metadata
float32 confidence
uint32 observation_count

# First and last seen timestamps
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
