# Package Name

## Overview

This is a ROS control warped interface for RoboMaster motor and some robot hardware
**Keywords:** ROS, RoboMaster

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Péter Fankhauser, pfankhauser@anybotics.com**

The rm_base package has been tested under [ROS] Melodic and Noetic on respectively 18.04 and 20.04. This is research
code, expect that it changes often and any fitness for a particular purpose is disclaimed.

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

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

  sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
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

### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

  Argument set 1

    - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

  Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.

#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...

#### Services

* **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

  	rosservice call /ros_package_template/get_average

#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.

### NODE_B_NAME

...

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues)
.


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[Eigen]: http://eigen.tuxfamily.org

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
