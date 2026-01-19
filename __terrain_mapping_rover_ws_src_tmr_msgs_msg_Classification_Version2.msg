# Single classification result from IMX500 or CPU model
std_msgs/Header header

string label              # Class label (e.g., "rock", "crater", "smooth_terrain")
float32 confidence        # Confidence score [0.0 - 1.0]
int32 bbox_x              # Bounding box top-left x (pixels)
int32 bbox_y              # Bounding box top-left y (pixels)
int32 bbox_width          # Bounding box width (pixels)
int32 bbox_height         # Bounding box height (pixels)