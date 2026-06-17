std_msgs/Header header
uint32 tracking_id
string class_id
float32 confidence

# Bounding box in pixel coordinates
float32 bbox_x        # center x
float32 bbox_y        # center y
float32 bbox_width
float32 bbox_height

#camera pose
geometry_msgs/Point position

geometry_msgs/Quaternion orientation
