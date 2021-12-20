#!/bin/bash
ros_distro=$1
ros_package_name=$2
ros_workspace=$3
tmp_space="/tmp/catkin_binary_deb"

source /opt/ros/$ros_distro/setup.bash
source $ros_workspace/devel/setup.bash

echo "Install packaging tool..."
sudo apt-get install python3-bloom fakeroot dh-make devscripts > /dev/null
pip install shyaml > /dev/null

time_stamp=`date +%Y%m%d.%H%M%S`
package_version="`curl -sL https://github.com/ros/rosdistro/raw/master/$ros_distro/distribution.yaml | shyaml get-value repositories.$ros_package_name.release.version`.$time_stamp"
run_directory=`pwd`

mkdir $tmp_space
cp -r ./ $tmp_space
package_list=`find $tmp_space -name package.xml | sed 's/package.xml//g'`
CM_PREFIX_PATH=`sed 's/:/;/g' <<< $CMAKE_PREFIX_PATH`

for package_source in $package_list
do
  echo "Trying to package $package_source in $package_version"
  cd $package_source
  bloom-generate rosdebian --os-name ubuntu --ros-distro $ros_distro
  debchange -v $package_version -p -D -u -m 'Append timestamp when binarydeb was built.'
  sed -e "s|-DCMAKE_PREFIX_PATH=.*|-DCMAKE_PREFIX_PATH=\""$CM_PREFIX_PATH"\"|g" -i debian/rules
  fakeroot debian/rules binary
  cd $run_directory
done
echo 'Package has been done.'
find $tmp_space -name '*.deb' -o -name '*.ddeb'|xargs -I {} cp {} $run_directory/
