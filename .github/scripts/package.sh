#!/bin/bash
ros_workspace=$1
ros_distro=$2
ros_package_name=$3
source /opt/ros/$ros_distro/setup.bash

sudo apt-get install python3-bloom fakeroot dh-make devscripts
pip install shyaml

package_version=`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.$ros_package_name.release.version`
time_stamp=`date +%Y%m%d.%H%M%S`
source $ros_workspace/devel/setup.bash
package_list=`find . -name package.xml | sed 's/package.xml//g' `

for package_source in $package_list
do
  if test -d $file
  then
    echo "Trying to package $package_source"
    cd $package_source
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd $GITHUB_WORKSPACE
  fi
done
echo 'Package has been done.'
find . -name '*.deb'
