# Status from VEX V5 brain
std_msgs/Header header

float32 battery_voltage   # Battery voltage (volts)
float32 battery_percent   # Battery percentage [0-100]
bool connected            # True if serial connection active
int32 left_encoder_ticks  # Raw left encoder ticks
int32 right_encoder_ticks # Raw right encoder ticks
float32 left_velocity     # Left wheel velocity (m/s)
float32 right_velocity    # Right wheel velocity (m/s)
