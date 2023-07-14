# rm-gazebo
## Overview

This package is to rewrite the Gazebo_ ros _control::RobotHWSim, adding the imu sensors.
The world file include the world of gazebo simulate.

**Keywords:** ros, gazebo, imu


### License

The source code is released under a [BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The rm_gazebo package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

![Example image](doc/example.jpg)


### Hardware interface type

+ `ImuSensorHandle` 
+ `RmImuSensorHandle` 
+ `RobotStateInterface`

## Installation

### Installation from Packages

To install all packages from this repository as Debian packages use

```shell
sudo apt-get install ros-noetic-rm-gazebo
```

or better use `rosdep`:

```shell
sudo rosdep install --from-paths src
```

## Config files

* **imus.yaml** the orientation covariance diagonal, angular velocity covariance and linear acceleration covariance config of imu.

* **mimic_joint.yaml** the pid gains are used by the gazebo mimic joint plugin.

* ****

## ROS API


#### Services

* **`switch_imu_status`** 


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


