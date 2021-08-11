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
- [rm_gazebo](https://github.com/gdut-dynamic-x/rm_gazebo) if `use_rm_gazbe`
- gazebo_ros
- gazebo_ros_control
- xacro

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:gdut-dynamic-x/rm_description.git
    # git clone https://github.com/gdut-dynamic-x/rm_description.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build # Actually nothing to build

## Usage

Run the simulation with:

	roslaunch rm_description hero.launch

## Config files

* **worlds/empty.worlds** Simulate physics eigen params.

## Launch files

* **hero.launch:** Launch Gazebo and load hero robot.

  Loading argument set

    - **`load_chassis`** Load chassis URDF. Default: `true`.
    - **`load_gimbal`** Load gimbal URDF. Default: `true`.
    - **`load_shooter`** Load shooter URDF. Default: `true`.
    - **`load_gazebo`** Load Gazebo params and run Gazebo. Default: `true`.
    - **`use_rm_gazebo`** Use Gazebo params and run Gazebo. Default: `false`.
    - **`paused`** Paused simulation when load Gazbeo. Default: `true`.

  Chassis argument set
    - **`roller_type`** How to simulate the roller of mecanum wheel, set `simple` to use sphere roller for speed up
      simulation, set `none` for real robot. Default: `realistic`(use one sphere with two
      capsule [roller.stl](meshes/common/roller.stl)).

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_description/issues)
.
