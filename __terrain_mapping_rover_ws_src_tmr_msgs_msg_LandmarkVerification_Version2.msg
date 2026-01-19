# Verification result comparing expected vs observed landmark position
std_msgs/Header header

string label              # Landmark label
string id                 # Landmark ID
float64 expected_x        # Expected position from map
float64 expected_y
float64 expected_z
float64 observed_x        # Observed position
float64 observed_y
float64 observed_z
float64 error_x           # Position error
float64 error_y
float64 error_z
float64 error_total       # Euclidean error magnitude
string status             # "OK", "DEGRADED", "CRITICAL"