#涉及到卡尔慢滤波器 用于估计位置

#储存着各种必须的数据v
std_msgs/Header header

#真是摄像头的位资数据v
geometry_msgs/Pose real_detection_pose

#滤波后的摄像头位资数据v
geometry_msgs/Pose filtered_detection_pose

#速度
geometry_msgs/Twist real_detection_twist

#速度
geometry_msgs/Twist filtered_detection_twist