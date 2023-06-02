uint8 RAW = 0
uint8 FOLLOW = 1
uint8 TWIST = 2
uint8 UP_SLOPE = 3
uint8 FALLEN = 4

uint8 mode
geometry_msgs/Accel accel
float64 power_limit
string follow_source_frame
string command_source_frame
time stamp
