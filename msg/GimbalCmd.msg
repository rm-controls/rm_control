# 这个msgs储存了 云台的各种命令

#云台的四个状态

#待机状态
uint8 PASSIVE = 0

#速度
uint8 RATE = 1

#追踪 自秒模式？
uint8 TRACK = 2

#直接操控？
uint8 DIRECT = 3

#时间戳
time stamp

#当前状态
uint8 mode

#??
uint8 target_id

#yaw旋转的速率
float64 rate_yaw

#绕x轴旋转的速率
float64 rate_pitch

#发射子弹的速度？
float64 bullet_speed

#包含air point 的包含坐标系和时间戳信息
geometry_msgs/PointStamped aim_point