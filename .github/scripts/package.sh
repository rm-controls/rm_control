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
source /opt/ros/noetic/setup.bash
source $ros_workspace/devel/setup.bash
$CM_PREFIX_PATH=sed 's/:/;/g' $CMAKE_PREFIX_PATH

for package_source in $package_list
do
  echo "Trying to package $package_source in $package_version"
  cd $package_source
  bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
  debchange -v $package_version -p -D -u -m 'Append timestamp when binarydeb was built.'
  sed -e "s|-DCMAKE_PREFIX_PATH=.*|-DCMAKE_PREFIX_PATH=\""$CM_PREFIX_PATH"\"|g" -i debian/rules
  fakeroot debian/rules binary
  cd $run_directory
done
echo 'Package has been done.'
find $ros_workspace/src -name '*.deb'|xargs -I {} cp {} $run_directory/
