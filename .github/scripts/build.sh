#!/bin/bash
ros_distro=$1

source /opt/ros/$ros_distro/setup.bash

package_list=`find $GITHUB_WORKSPACE/ -name package.xml | sed 's/package.xml//g' `  
mkdir -p /tmp/catkin_ws/src && cd /tmp/catkin_ws 
catkin_init_workspace /tmp/catkin_ws/src
for package_source in $package_list
do
    cp -r $package_source /tmp/catkin_ws/src
done
rosdep update
rosdep install --from-paths /tmp/catkin_ws/src --ignore-packages-from-source --rosdistro $ros_distro -y
catkin_make -C /tmp/catkin_ws
echo "::set-output name=catkin-ws-directory::$(echo /tmp/catkin_ws)"