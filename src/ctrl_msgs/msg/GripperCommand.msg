# Parallel gripper width command.
std_msgs/Header header

# Width is the distance between finger inner surfaces in meters.
bool control_left
float64 left_width_m

bool control_right
float64 right_width_m

float64 velocity_mps 0.05
float64 effort 10.0
