# Package Name

## Overview

This package includes srv, action, msgs and other message sending mechanisms, including the message types needed in
various packages.
**Keywords:** message, mechanisms

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [Dynamicx]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The rm_hw package has been tested under [ROS] Melodic and Noetic on respectively 18.04 and 20.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

![Example image](doc/example.jpg)

[comment]: <> (### Publications)

[comment]: <> (If you use this work in an academic context, please cite the following publication&#40;s&#41;:)

[comment]: <> (* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference)

[comment]: <> (  on Intelligent Robots and Systems &#40;IROS&#41;, 2015. &#40;[PDF]&#40;http://dx.doi.org/10.3929/ethz-a-010173654&#41;&#41;)

[comment]: <> (        @inproceedings{Fankhauser2015,)

[comment]: <> (            author = {Fankhauser, P\'{e}ter and Hutter, Marco},)

[comment]: <> (            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems &#40;IROS&#41;},)

[comment]: <> (            title = {{PAPER TITLE}},)

[comment]: <> (            publisher = {IEEE},)

[comment]: <> (            year = {2015})

[comment]: <> (        })

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/rm_msgs.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

[comment]: <> (### Running in Docker)

[comment]: <> (Docker is a great way to run an application with all dependencies and libraries bundles together. Make sure)

[comment]: <> (to [install Docker]&#40;https://docs.docker.com/get-docker/&#41; first.)

[comment]: <> (First, spin up a simple container:)

[comment]: <> (	docker run -ti --rm --name ros-container ros:noetic bash)

[comment]: <> (This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal &#40;`-t, -i`&#41;)

[comment]: <> (, gives it a name &#40;`--name`&#41;, removes it after you exit the container &#40;`--rm`&#41; and runs a command &#40;`bash`&#41;.)

[comment]: <> (Now, create a catkin workspace, clone the package, build it, done!)

[comment]: <> (	apt-get update && apt-get install -y git)

[comment]: <> (	mkdir -p /ws/src && cd /ws/src)

[comment]: <> (	git clone https://github.com/leggedrobotics/ros_best_practices.git)

[comment]: <> (	cd ..)

[comment]: <> (	rosdep install --from-path src)

[comment]: <> (	catkin_make)

[comment]: <> (	source devel/setup.bash)

[comment]: <> (	roslaunch ros_package_template ros_package_template.launch)

## Packages

### msgs

#### ActuatorState.msgs

This message contains the state of An actuator contains a motor and an encoder, and is connected to a joint by a
transmission.

- time[] stamp

  The time at which this actuator state was measured

- string[] name

  The name of the actuator

- string[] type

  The type of the actuator

- string[] bus

  The CAN bus

- int32[] id

  The CAN id

- bool[] halted

  Indicates if the motor is halted. A motor can be halted because of a voltage or temperature problem

- bool[] need_calibration

  calibration

- bool[] calibration_reading

  The value of the calibration reading: low (false) or high (true)

- uint16[] position_raw

  The encoder raw position, represented by the number of encoder ticks

- int16[] velocity_raw

  The encoder velocity, represented by rpm

- uint8[] temperature

  The temperature of the motor, represented by c1elsius

- int64[]  circle

  The circle of absolute encoder

- uint16[] last_position_raw

  The last encoder raw position, represented by the number of encoder ticks

- float64[]  frequency

  The encoder frequency

- float64[] position

  The encoder position in radians

- float64[] velocity

  The encoder velocity in radians per second

- float64[] effort

  The last effort that was measured by the actuator

- float64[] commanded_effort

  The last effort command that was requested without limit

- float64[] executed_effort

  The last effort command that was requested with limit

- float64[] offset

  The angular offset (in radians) that is added to the encoder reading, to get to the position of the actuator. This
  number is computed when the referece sensor is triggered during the calibration phase

#### BalanceState.msg

This message contains the state of balance robot.

- std_msgs/Header header Standard metadata for higher-level stamped data types. This is generally used to communicate
  timestamped data in a particular coordinate frame.

- float64 alpha

  the angle alpha of robot.

- float64 alpha_dot

  the angle alpha speed of robot.

- float64 vel

  the velocity of robot

- float64 theta_dot

  the angle theta speed of robot.

- float64 control_1

  left joint control

- float64 control_2

  right joint control

#### ChassisCmd.msg

This message contains various state parameter settings for basic chassis control

- uint8 RAW = 0

  the mode RAW:the Initial state.

- uint8 FOLLOW = 1

  the mode FOLLOW

- uint8 GYRO = 2

  the mode GYRO

- uint8 TWIST = 3

  the mod TWIST:

- uint8 mode

  The current mode of the chassis.

- geometry_msgs/Accel accel

  This expresses acceleration in free space broken into its linear and angular parts.

- float64 power_limit

  The Current power limit on the chassis

- string follow_source_frame

  The source coordinate system tracked by the current chassis

- time stamp

  The time at which this actuator state was measured

#### DbusData.msgs

This message contains the information needed for remote control operation

- UP DOWN MID

  Set various operating modes of the robot from the trackwheel

- ch_l/r-x/y

  Operation of the left and right joysticks

- s_l/r

  Operation of the left and right impellers

- wheel

  The wave wheel that controls the small top

- m/p_xyz/lr

  Various mouse operations

- key_

  Various operations of the keyboard

- time stamp

  The time at which this actuator state was measured

#### GimbalCmd.msg

This message includes various commands received by the gimbal

- uint8 RATE

Bullet launch

- uint8 TRACK

  Self-aim

- uint8 DIRECT

  Manual control

- time stamp

  The time at which this actuator state was measured

- uint8 mode

  The current mode of the chassis.

- uint8 target_id


- float64 rate_yaw

- float64 rate_pitch


- float64 bullet_speed

  Bullet firing speed

- geometry_msgs/PointStamped

  This represents a Point with reference coordinate frame and timestamp

#### GimbalDesError.msg

#### KalmanData.msg

This message contains various kinematics data

- std_msgs/Header header

  Standard metadata for higher-level stamped data types. This is generally used to communicate timestamped data in a
  particular coordinate frame.

- geometry_msgs/Pose real_detection_pose

  A representation of pose in free space, composed of position and orientation

- geometry_msgs/Pose filtered_detection_pose

  A representation of pose in free space, composed of position and orientation

- geometry_msgs/Twist real_detection_twist

  This expresses velocity in free space broken into its linear and angular parts.

- geometry_msgs/Twist filtered_detection_twist

  This expresses velocity in free space broken into its linear and angular parts.

#### LpData.msg

#### MovingAverageData.msg

#### Referee.msg

#### shootCmd.msg

#### superCapacitor.msg

### srv

### action

[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
