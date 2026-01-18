# Single BLE beacon signal
std_msgs/Header header

string uuid
uint16 major
uint16 minor

int8 rssi          # Signal strength (dBm)
float32 distance   # Estimated distance (m)
float32 accuracy   # Distance accuracy

uint32 measurement_count