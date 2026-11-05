# Omni-wheel velocity command.
std_msgs/Header header

# Angular velocity in radians/second for wheel1_joint through wheel4_joint.
float64[4] wheel_velocity

# Command lease in seconds. The real backend stops base motion when commands
# stop arriving for this long.
float64 timeout_s 0.5
