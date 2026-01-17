# VEX V5 Brain Status Message
std_msgs/Header header

# Connection status
bool connected
uint32 message_count
uint32 error_count

# Battery info (if available)
float32 battery_voltage
float32 battery_percentage

# Motor temperatures (Celsius)
float32[4] motor_temperatures

# Communication stats
float32 update_rate_hz
uint32 checksum_errors
