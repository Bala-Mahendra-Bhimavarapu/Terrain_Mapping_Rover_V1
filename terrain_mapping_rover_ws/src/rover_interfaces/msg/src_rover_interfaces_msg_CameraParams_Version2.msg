# Camera parameter control
std_msgs/Header header

# Auto exposure control
bool ae_enabled
float32 target_brightness  # 0-255

# Manual parameters (used when ae_enabled=false)
float32 exposure_time_us
float32 analogue_gain
float32 digital_gain

# ROI (normalized 0-1)
float32 roi_x
float32 roi_y
float32 roi_width
float32 roi_height

# Limits
float32 min_exposure_us
float32 max_exposure_us
float32 min_gain
float32 max_gain