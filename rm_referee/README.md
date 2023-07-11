# Package Name

## Overview

This is a ROS control warped interface for RoboMaster motor and some robot hardware
**Keywords:** ROS, RoboMaster, referee

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [Dynamicx]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The rm_referee package has been tested under [ROS] Melodic and Noetic on respectively 18.04 and 20.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [rm_msg](https://github.com/gdut-dynamic-x/rm_msgs)
- [rm_common](https://github.com/gdut-dynamic-x/rm_common)
- sensor_msgs
- roscpp
- serial
- tf2_geometry_msgs
- std_msgs
- actionlib
- nav_msgs



#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/rm-controls/rm_control.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


### Static code analysis

Run the static code analysis with

	catkin build rm_referee

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch rm_referee load.launch

## Config files

rm_config/config/rm_referee/engineer.yaml

* **engineer.yaml:** Set the base UI of engineer

rm_config/config/rm_referee/hero.yaml

* **hero.yaml:** Set the base UI of hero

rm_config/config/rm_referee/radar.yaml

* **radar.yaml:** Set the config of radar

rm_config/config/rm_referee/standard3.yaml
rm_config/config/rm_referee/standard4.yaml
rm_config/config/rm_referee/standard5.yaml

* **standard.yaml:** Set the base UI of standard

## Launch files

* **load.launch:** Load the robot config in config files

  robot type

    - **`robot_type`** set the robot type. Default: `$(env ROBOT_TYPE)`.

  node

    - **`node`** set the node name.


## Nodes

### rm_referee

Load the setting config of UI. 

#### Subscribed Topics

* **`/joint_states`** ([sensor_msgs/joint_states])

 get the states of each joints

* **`/actuator_states`** ([sensor_msgs/actuator_states])

 get the states of actuator

* **`/dbus_data`** ([sensor_msgs/dbus_data])

 get the date of dbus

 * **`/controllers/chassis_controller/command`** ([/controllers/chassis_controller/command])

 get the states of chassis from controllers

 * **`/controllers/shooter_controller/state`** 

 get the states of shooter

 * **`/cmd_vel`** 

 get the vel of robot

 * **`/controllers/gimbal_controller/command`**

 get the states of gimbal

 * **`/engineer_ui`** 

 get the states of UI set for engineer

 * **`/manual_to_referee`** 

 get the date of manual

 * **`/pnp_publisher`** 

 get the date of exchange 

 * **`/planning_result`** 

 get the planning_result


#### Published Topics

 * **`super_capacitor`** 

 pub the date of super_catpacitor

 * **`game_robot_status`** 

 pub the date of robot

  * **`game_status`** 

 pub the date of game

  * **`capacity_data`** 

 pub the date of capacity

  * **`power_heat_data`** 

 pub the date of power heat

  * **`game_robot_hp`** 

 pub the date of  robot hp

  * **`super_capacitor`** 

 pub the date of super_catpacitor

  * **`event_data`** 

 pub the date of event 

  * **dart_status_data`** 

 pub the date of dart status

  * **`icra_buff_debuff_zone_status_data`** 

 pub the date of sicra buff debuff zone status 

  * **`supply_projectile_action_data`** 

 pub the date of supply_projectile_action

  * **`dart_remaining_time_data`** 

 pub the date of dart_remaining_time

  * **`robot_hurt_data`** 

 pub the date of robot hurt

  * **`shoot_data`** 

 pub the date of shoot

  * **`bullet_allowance_data`** 

 pub the date of bullet_allowance 

  * **`rfid_status_data`** 

 pub the date of rfid_status

  * **`dart_client_cmd_data`** 

 pub the date of dart_client_cmd 

  * **`client_map_receive`** 

 pub the date of client map 

  * **`robot_position"`** 

 pub the date of robot_position"

  * **`radar_mark`** 

 pub the date of radar_mark

   * **`client_map_send_data`** 

 pub the date of client_map_send

 

#### Services

...

#### Parameters

* **`robot_type`** (string, default: "$(env ROBOT_TYPE)")

  The name of the robot.

* **`ui`** 

  The state of every kinds of ui
  
* **`trigger_change`** 

  The state of every kinds of trigger change ui

* **`fixed`** 

  The state of every fixed ui
  
### dbus_node

...

## Plugins

### robot_state_controller

#### Subscribed Topics

...

#### Published Topics

...

### RevoluteTransmission

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
