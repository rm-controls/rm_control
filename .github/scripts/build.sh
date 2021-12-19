#!/bin/bash
ros_distro=$1
ros_workspace='/tmp/catkin_ws'
source /opt/ros/$ros_distro/setup.bash

package_list=`find $GITHUB_WORKSPACE/ -name package.xml | sed 's/package.xml//g'`
mkdir -p $ros_workspace/src
for package_source in $package_list
do
    cp -r $package_source $ros_workspace/src
done
rosdep update
rosdep install --from-paths $ros_workspace/src --ignore-packages-from-source --rosdistro $ros_distro -y
catkin_make -C $ros_workspace
catkin_make -C $ros_workspace install
echo "::set-output name=catkin-ws-directory::$(echo $ros_workspace)"
