uint8 RATE = 0
uint8 TRACK = 1
uint8 DIRECT = 2

time stamp
uint8 mode

# RATE
float64 rate_yaw
float64 rate_pitch

# TRACK/DIRECT
float64 bullet_speed
geometry_msgs/PointStamped target_pos
geometry_msgs/Vector3Stamped target_vel
