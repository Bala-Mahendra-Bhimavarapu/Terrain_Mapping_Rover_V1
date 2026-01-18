# Tank Drive Command Message
std_msgs/Header header

# Left and right velocities in m/s
float64 left_velocity
float64 right_velocity

# Optional: direct motor power (-1.0 to 1.0)
# Set use_velocity=false to use power instead
bool use_velocity
float64 left_power
float64 right_power

# Emergency stop flag
bool emergency_stop