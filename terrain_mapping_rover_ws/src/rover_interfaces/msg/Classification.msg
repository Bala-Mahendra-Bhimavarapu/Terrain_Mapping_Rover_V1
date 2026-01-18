# Single classification result
std_msgs/Header header

string label
float32 confidence
uint32 class_id

# Bounding box (pixels)
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height

# Image dimensions
uint32 image_width
uint32 image_height
