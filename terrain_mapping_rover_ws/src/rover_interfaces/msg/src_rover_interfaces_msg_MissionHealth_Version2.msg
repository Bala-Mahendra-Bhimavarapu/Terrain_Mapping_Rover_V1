# Overall mission health status
std_msgs/Header header

# Status: OK, DEGRADED, CRITICAL
string status

# Subsystem health (OK, DEGRADED, CRITICAL, OFFLINE)
string slam_status
string localization_status
string navigation_status
string perception_status
string motor_status

# Key metrics
float32 slam_quality          # 0-1, loop closures, features
float32 localization_confidence  # 0-1, from EKF covariance
float32 vo_feature_count      # Visual odometry features
float32 loop_closure_count    # Recent loop closures

# Covariance trace (uncertainty)
float64 position_uncertainty
float64 orientation_uncertainty

# Resource usage
float32 cpu_usage
float32 memory_usage
float32 disk_usage