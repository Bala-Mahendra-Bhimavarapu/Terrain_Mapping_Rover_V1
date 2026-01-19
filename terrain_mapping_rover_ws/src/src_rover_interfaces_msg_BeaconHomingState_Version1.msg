# Beacon Homing State Message
# Reports current state of autonomous beacon homing

std_msgs/Header header

# Enable state
bool enabled

# Current state (SEARCHING, APPROACHING, AVOIDING, ARRIVED, LOST)
string state

# Beacon tracking
bool beacon_detected
float32 beacon_distance          # meters (-1 if not detected)
int32 beacon_rssi                # dBm
bool signal_improving            # Is signal getting stronger?
float32 estimated_direction      # Direction to beacon (radians, robot frame)
float32 confidence               # Tracking confidence (0-1)

# Obstacle detection
bool obstacle_detected
float32 obstacle_distance        # meters to nearest obstacle

# Current command
float32 linear_velocity          # m/s
float32 angular_velocity         # rad/s

# Status message
string message