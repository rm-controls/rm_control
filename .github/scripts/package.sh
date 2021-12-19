#!/bin/bash
ros_package_name=$1

sudo apt-get install python3-bloom fakeroot dh-make devscripts
pip install shyaml

package_version=`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.$ros_package_name.release.version`
time_stamp=`date +%Y%m%d.%H%M%S`
root_directory=`pwd`
package_list=`find . -name package.xml | sed 's/package.xml//g' `

for package_source in $package_list
do
  echo "Trying to package $package_source"
  cd $package_source
  bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
  debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
  fakeroot make -f debian/rules binary
  cd $root_directory
done
echo 'Package has been done.'
find . -name '*.deb'
