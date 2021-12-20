#!/bin/bash
ros_workspace=$1
ros_package_name=$2

echo "Install packaging tool..."
sudo apt-get install python3-bloom fakeroot dh-make devscripts > /dev/null
pip install shyaml > /dev/null

time_stamp=`date +%Y%m%d.%H%M%S`
package_version="`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.$ros_package_name.release.version`.$time_stamp"
run_directory=`pwd`
package_list=`find $ros_workspace/src -name package.xml | sed 's/package.xml//g' `
source $ros_workspace/devel/setup.bash

for package_source in $package_list
do
  echo "Trying to package $package_source in $package_version"
  cd $package_source
  bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
  debchange -v $package_version -p -D -u -m 'Append timestamp when binarydeb was built.'
  sed -i "s:-DCMAKE_PREFIX_PATH=.*:-DCMAKE_PREFIX_PATH=\"$CMAKE_PREFIX_PATH\":g" debian/rules
  fakeroot make -f debian/rules binary
  cd $run_directory
done
echo 'Package has been done.'
find $ros_workspace/src -name '*.deb'|xargs -I {} cp {} $run_directory/
