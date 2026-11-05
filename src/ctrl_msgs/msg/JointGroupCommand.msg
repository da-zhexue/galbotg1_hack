# Joint-space command for one body group.
std_msgs/Header header

# Joint names in the same order as position, velocity, and effort.
string[] name

# Target joint positions in radians.
float64[] position

# Optional feedback values for visualization. Leave empty or use one value
# per joint.
float64[] velocity
float64[] effort
