# Current exposure state feedback
std_msgs/Header header

float32 current_brightness  # 0-255 measured
float32 target_brightness   # 0-255 target
float32 exposure_time_us
float32 analogue_gain
float32 digital_gain

bool ae_locked
bool motion_detected
string adjustment_state  # STABLE, ADJUSTING, LOCKED
