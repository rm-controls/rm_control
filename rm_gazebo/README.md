# rm_gazebo


## Overview

This package is to modify the class of `Gazebo_ros_control::RobotHWSim`, adding the imu sensors hardware interface for gazebo robot hardware simulation.
The world folder contains the gazebo simulate world.

**Keywords:** ros, gazebo, imu, hardware interface


### License

The source code is released under a [BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The rm_gazebo package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)



### Hardware interface type

+ `ImuSensorHandle` 
+ `RmImuSensorHandle` 
+ `RobotStateInterface`

## Installation

### Dependencies

- roscpp
- rm_common
- gazebo
- gazebo_ros
- gazebo_ros_control
- rm_description<
- roboticsgroup_upatras_gazebo_plugins

### Installation from Packages

To install all packages from this repository as Debian packages use

```shell
sudo apt-get install ros-noetic-rm-gazebo
```

or better use `rosdep`:

```shell
sudo rosdep install --from-paths src
```

### Config files

* **imus.yaml** the orientation covariance diagonal, angular velocity covariance and linear acceleration covariance config of imu.

* **mimic_joint.yaml** the pid gains are used by the gazebo mimic joint plugin.

* ****

### Launch files

* **big_resource.launch:** launch the simulate with the world of big resource.

* **empty_world.launch:**launch the simulate with the world of empty world.

* **exchange_station.launch:**launch the simulate with the world of exchange station.

* **rmuc.launch.launch:**launch the simulate with the world for rmuc.

* **sentry_world.launch:**launch the simulate with the world for sentry.

* **small_resource.launch:**launch the simulate with the world of small_resource.

* **stone.launch:**launch the simulate with the world of a stone.

* **small_resource.launch:**launch the simulate with the world of warthog race .


### Services

* **`switch_imu_status`**  control the imu status and send the imu status message

### Parameters

* **imus** (`xml_rpc_value`)

#### default orientation
* **orientation_covariance_diagonal**
$$
 \left[
 \begin{matrix}
   0.0012 & 0 & 0 \\
   0 & 0.0012 & 0 \\
   0 & 0 & 0.0012
  \end{matrix}
  \right] \tag{3}
$$


* **angular_velocity_covariance**
$$
 \left[
 \begin{matrix}
   0.0004 & 0 & 0 \\
   0 & 0.0004 & 0 \\
   0 & 0 & 0.0004
  \end{matrix}
  \right] \tag{3}
$$

* **linear_acceleration_covariance**
$$
 \left[
 \begin{matrix}
   0.01 & 0 & 0 \\
   0 & 0.01 & 0 \\
   0 & 0 & 0.01
  \end{matrix}
  \right] \tag{3}
$$

### Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


