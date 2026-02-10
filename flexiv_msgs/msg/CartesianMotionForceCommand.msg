# Cartesian motion-force command for Flexiv robots
# Used with RT_CARTESIAN_MOTION_FORCE mode (real-time Cartesian motion-force control)

std_msgs/Header header

# Target TCP pose in world frame [x, y, z, qw, qx, qy, qz]
geometry_msgs/Pose target_pose

# Target TCP wrench (force and moment) in force control reference frame
# [fx, fy, fz, mx, my, mz] Unit: [N]:[Nm]
geometry_msgs/Wrench target_wrench

# Target TCP velocity (linear and angular) in world frame
# [vx, vy, vz, wx, wy, wz] Unit: [m/s]:[rad/s]
geometry_msgs/Twist target_velocity

# Target TCP acceleration (linear and angular) in world frame
# [ax, ay, az, alpha_x, alpha_y, alpha_z] Unit: [m/s^2]:[rad/s^2]
geometry_msgs/Accel target_acceleration
