# This message contains the state of an actuator
# An actuator contains a motor and an encoder, and is connected
# to a joint by a transmission

# The name of the actuator
string name

# The CAN bus
string bus

# The CAN id
int32 id

# The time at which this actuator state was measured
time timestamp

# The encoder raw position, represented by the number of encoder ticks
uint16 position_raw

# The encoder velocity, represented by rpm
uint16 velocity_raw

# The temperature of the motor, represented by c1elsius
uint8 temperature

# The circle of absolute encoder
int64  circle

# The last encoder raw position, represented by the number of encoder ticks
uint16 last_position_raw



# The encoder position in radians
float64 position

# The encoder velocity in radians per second
float64 velocity

# The last effort that was measured by the actuator
float64 effort

# The angular offset (in radians) that is added to the encoder reading,
# to get to the position of the actuator. This number is computed when the referece
# sensor is triggered during the calibration phase
float64 offset

# Is calibrating
bool is_calibrating

# The value of the calibration reading: low (false) or high (true)
bool calibration_reading

# Indicates if the motor is halted. A motor can be halted because of a voltage or temperature problem
bool halted
