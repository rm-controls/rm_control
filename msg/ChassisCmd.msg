#这是一个用来存放地盘控制命令的message

#未开启状态
uint8 PASSIVE = 0

#原状态 未执行任何命令
uint8 RAW = 1

#自秒模式？
uint8 FOLLOW = 2

#正常前后左右行走模式
uint8 TWIST = 3

#小陀螺模式
uint8 GYRO = 4

#保存机器人所处的状态
uint8 mode

#加速度
geometry_msgs/Accel accel

#功率限制
float64 power_limit

# 当前的 坐标系？
string follow_source_frame

# The time at which this actuator state was measured
time stamp