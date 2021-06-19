#基本头
std_msgs/Header header

#真是的位资与速度
float64 real_pos_x
float64 real_pos_y
float64 real_pos_z
float64 real_vel_x
float64 real_vel_y
float64 real_vel_z

#滤波处理之后的位资和速度
float64 filtered_pos_x
float64 filtered_pos_y
float64 filtered_pos_z
float64 filtered_vel_x
float64 filtered_vel_y
float64 filtered_vel_z
float64 filtered_center_x
float64 filtered_center_y
float64 filtered_center_z

#脱落一
float64 real_gyro_vel
float64 filtered_gyro_vel