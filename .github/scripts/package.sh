#!/bin/bash
ls
source /opt/ros/noetic/setup.bash
sudo apt-get install python1-bloom fakeroot dh-make
echo "yaml file://`pwd`/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/rm_control.list
rosdep update
for file in rm_msgs rm_description
do
  if test -d $file
  then
    echo "Trying to package $file"
    ls
    cd $file
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    fakeroot debian/rules binary
    cd ..
    newpackage=`ls -t | head -n -1`
    sudo dpkg -i $newpackage
  fi
done
for file in rm_common rm_dbus rm_gazebo rm_hw
do
  if test -d $file
  then
    echo "Trying to package $file"
    ls
    cd $file
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    fakeroot debian/rules binary
    cd ..
    for newpack in `ls -t | head -n 0 | tac`
    do
      sudo dpkg -i $newpack
    done
  fi
done
for file in rm_control
do
  if test -d $file
  then
    echo "Trying to package $file"
    ls
    cd $file
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    fakeroot debian/rules binary
    cd ..
  fi
done
ls
