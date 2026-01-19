# 2D landmark detection from YOLO
std_msgs/Header header

string label              # Class label
float32 confidence        # Detection confidence [0.0 - 1.0]
int32 bbox_x              # Bounding box top-left x
int32 bbox_y              # Bounding box top-left y
int32 bbox_width          # Bounding box width
int32 bbox_height         # Bounding box height
bool is_stationary        # True if classified as a landmark (not moving obstacle)
bool is_obstacle          # True if should be avoided
