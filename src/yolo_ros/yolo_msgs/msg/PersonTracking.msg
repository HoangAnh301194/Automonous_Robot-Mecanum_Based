# defines a simplified tracking result for Behavior Trees
std_msgs/Header header

# ID for tracking
string id

# 2D coordinates in the image frame (pixels)
float64 u
float64 v

# 3D distance in meters (from camera to person)
float64 distance

# 3D position in base_link (x=forward, y=left, z=up)
geometry_msgs/Point position_3d
