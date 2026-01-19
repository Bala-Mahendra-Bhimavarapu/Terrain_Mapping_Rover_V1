# Overall mission health status
std_msgs/Header header

string status             # "OK", "DEGRADED", "CRITICAL", "EMERGENCY_STOP"
string[] active_issues    # List of current issues
float32 slam_confidence   # SLAM confidence [0.0 - 1.0]
float32 localization_confidence  # EKF confidence [0.0 - 1.0]
int32 loop_closure_count  # Number of loop closures
int32 vo_feature_count    # Visual odometry feature count
float32 battery_voltage   # VEX V5 battery voltage
bool emergency_stop_active  # True if e-stop is engaged