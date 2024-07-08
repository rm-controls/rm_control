uint8 RATE = 0
uint8 TRACK = 1
uint8 DIRECT = 2
uint8 TRAJ = 3

time stamp
uint8 mode

# RATE
float64 rate_yaw
float64 accel_yaw
float64 rate_pitch
float64 accel_pitch

# TRACK/DIRECT
float64 bullet_speed
geometry_msgs/PointStamped target_pos

#TRAJ
float64 yaw_des
float64 pitch_des