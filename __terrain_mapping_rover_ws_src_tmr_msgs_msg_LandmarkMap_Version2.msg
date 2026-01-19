# Complete landmark map for storage/transfer
std_msgs/Header header

string map_id             # Unique map identifier
string rover_id           # Rover that created this map
tmr_msgs/LandmarkPosition[] landmarks
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at