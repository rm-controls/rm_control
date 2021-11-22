#!/bin/bash
ls
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-bloom fakeroot dh-make devscripts
pip install shyaml
package_version=`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.rm_control.release.version`
time_stamp=`date +%Y%m%d.%H%M%S`
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
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd ..
    sudo dpkg -i `ls -t | head -n 1`
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
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd ..
    sudo dpkg -i `ls -t | head -n 2 | tac`
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
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd ..
  fi
done
ls
