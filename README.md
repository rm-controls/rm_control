# rm_description

## Overview

This is a ROS package with description files of RoboMaster robot made by DynamicX.

**Keywords:** RoboMaster, URDF, description

Or, add some keywords to the Bitbucket or GitHub repository.

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [DynamicX]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The rm_description package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.

![Example image](doc/hero_chassis_only_gazebo.png)

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- gazebo_ros
- gazebo_ros_control
- xacro

Install dependencies:

    sudo rosdep install rm_description

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:gdut-dynamic-x/rm_description.git
    # git clone https://github.com/gdut-dynamic-x/rm_description.git
	cd ../
	catkin build # Actually nothing to build

## Usage

Run the simulation with:

	roslaunch rm_description hero_chassis_only.launch

## Config files

* **worlds/empty.worlds** Simulate physics eigen params.

## Launch files

* **hero_chassis_only.launch:** Launch Gazebo and load hero robot with chassis only.

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_description/issues)
.
