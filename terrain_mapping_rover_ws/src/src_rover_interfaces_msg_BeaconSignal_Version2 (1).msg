# Single Bluetooth beacon signal reading
std_msgs/Header header

string uuid                         # Beacon UUID
uint16 major                        # Major value
uint16 minor                        # Minor value
int32 rssi                          # Signal strength (dBm, typically -30 to -100)
int32 tx_power                      # Transmitted power at 1m (dBm)
float32 distance                    # Estimated distance (meters)
float32 accuracy                    # Distance accuracy estimate (meters)
uint32 measurement_count            # Number of measurements averaged