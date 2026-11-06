# Difference between the latest real and simulated robot states.
std_msgs/Header header

string[] joint_names
float64[] joint_errors
float64 max_joint_error
string max_joint_name

bool left_end_effector_valid
float64 left_position_error_m
float64 left_orientation_error_rad

bool right_end_effector_valid
float64 right_position_error_m
float64 right_orientation_error_rad
