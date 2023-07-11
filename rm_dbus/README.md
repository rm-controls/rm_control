# Rm_dbus

## Overview

The rm_dbus package can read and unpack data transmitted through
the serial port after pressing a button.Then assign true or false
to the key variable.

**Keywords:** read, unpack, key variable

### License

The source code is released under a [BSD 3-Clause license](https://github.com/rm-controls/rm_control/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The rm_dbus package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-rm-dbus

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- roscpp
- rm_common

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/rm-controls/rm_control.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch rm_dbus rm_dbus

## Config files

dbus.yaml

* **`serial_port`** Set up serial port.

## Nodes

### rm_dbus

Keep running the run function, that is, keep running read(), unpack(), and getdata().

#### Published Topics

* **`/dbus_data`** ([rm_msgs/DbusData])

  The dbus data used to execute the corresponding action. The corresponding action write in rm_manual.

#### Parameters

* **`serial_port`** (string, default: "/dev/usbDbus")

  Set up serial port.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_control/issues)
.


[ROS]: http://www.ros.org

[rm_msgs/DbusData]: https://github.com/rm-controls/rm_control/blob/master/rm_msgs/msg/DbusData.msg
