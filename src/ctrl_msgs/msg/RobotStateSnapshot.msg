# Unified real-robot or simulator state snapshot.
std_msgs/Header header

# Source name: "real" or "sim".
string source
string reference_frame

# Joint angles in radians.
string[] joint_names
float64[] joint_positions

# End-effector poses in reference_frame.
string left_end_effector_frame
bool left_end_effector_valid
geometry_msgs/Pose left_end_effector_pose

string right_end_effector_frame
bool right_end_effector_valid
geometry_msgs/Pose right_end_effector_pose
