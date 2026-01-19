# Bluetooth beacon detection
std_msgs/Header header

string uuid               # Beacon UUID
int32 rssi                # Signal strength (dBm, typically -100 to 0)
float32 estimated_distance  # Estimated distance (meters), if calculable
bool beacon_reached       # True if RSSI > threshold