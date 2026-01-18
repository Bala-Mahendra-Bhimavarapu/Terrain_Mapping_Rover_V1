# Raw Wheel Odometry from VEX V5
std_msgs/Header header

# Encoder ticks (cumulative)
int64 left_ticks
int64 right_ticks

# Encoder velocities (ticks/sec)
float64 left_ticks_per_sec
float64 right_ticks_per_sec

# Computed velocities (m/s)
float64 left_velocity
float64 right_velocity

# Computed pose (in odom frame)
float64 x
float64 y
float64 theta

# Timestamp from VEX (ms since start)
uint64 vex_timestamp_ms
