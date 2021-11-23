#!/bin/bash
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-bloom fakeroot dh-make devscripts
pip install shyaml
package_version=`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.rm_control.release.version`
time_stamp=`date +%Y%m%d.%H%M%S`
#echo "yaml file://`pwd`/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/rm_control.list
rosdep update
for package in `ls`
do
  if test -e $package/package.xml
  then
    echo "Trying to package $package"
    cd $package
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd ..
    sudo dpkg -i `ls -t | grep .deb | head -n 1`
  fi
done
echo 'Package is Done.'
ls | grep .deb
ls | grep .ddeb
