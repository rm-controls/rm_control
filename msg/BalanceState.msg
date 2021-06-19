## 这个msg包括了 平衡步兵机器人的 状态 也就是说 描绘了平衡步兵机器人的状态


# header中包括三个元素 编号 时间戳 以及 坐标系名称
std_msgs/Header header

#指的是机器人地盘偏移的角度？  指的是机器人前后摆的角度
float64 alpha

#指的是偏移时候的角速度
float64 alpha_dot

#地盘运动的线速度
float64 vel

#某一个角度的角速度   指的是旋转的角度角速度  ya
float64 theta_dot

#指的是左轮得到的命令？
float64 control_1

#指的是右轮得到的命令
float64 control_2