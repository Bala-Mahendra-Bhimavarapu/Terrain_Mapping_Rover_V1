# 3D landmark position in map frame
std_msgs/Header header

string label              # Class label
string id                 # Unique landmark ID
float64 x                 # Position in map frame (meters)
float64 y
float64 z
float64[9] covariance     # 3x3 covariance matrix (row-major)
float32 confidence        # Detection confidence
uint32 observation_count  # Number of times observed
