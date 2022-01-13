^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2021-12-7)
------------------
* Merge branch 'master' into master
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#14 <https://github.com/rm-controls/rm_control/issues/14>`_ from CQUMechaX/master
  Fix rm_msgs generation problem on clean make
* Fix rm_msgs generation problem on clean make
  When you use catkin_make with make -jxx, rm_msgs may be compiled later than
  targets which need it. It will throw an error on a clean workspace and works
  perfectly later on.
  - See https://answers.ros.org/question/73048
* Make rm_manual can be used with gimbal controller in gimbal/opti_simplify branch.
* Merge branch 'master' into gimbal/opti_or_simplify
* Update CHANGELOG
* Merge remote-tracking branch 'origin/gimbal/opti_or_simplify' into gimbal/opti_or_simplify
* Put filtered quaternion into imu_extra_handle.
* Add setOrientation to ImuExtraHandle
* Add orientation to ImuExtraHandle
* Add ImuExtraInterface
* Contributors: BruceLannn, QiayuanLiao, Tiger3018, YuuinIH, qiayuan

0.1.7 (2021-09-26)
------------------
* 0.1.6
* Update CHANGELOG
* Merge branch 'namespace' into rm_gazebo/imu_sensor_interface
* Merge pull request `#8 <https://github.com/rm-controls/rm_control/issues/8>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control.
* Change name of namespace:from hardware_interface to rm_control.
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.6 (2021-09-26)
------------------
* Merge branch 'namespace' into rm_gazebo/imu_sensor_interface
* Merge pull request `#8 <https://github.com/rm-controls/rm_control/issues/8>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control.
* Change name of namespace:from hardware_interface to rm_control.
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.5 (2021-09-02)
------------------

0.1.4 (2021-09-02)
------------------

0.1.3 (2021-09-01)
------------------
* Format the code
* Format the code
* Add licence
* Merge branch 'rm-controls:master' into master
* Merge pull request `#7 <https://github.com/rm-controls/rm_control/issues/7>`_ from Peter-Chan-tech/master
  Move referee operations to rm_common
* Remove serial lib in rm_common
* Reformat
* Add new line at the end of files
* Move referee.cpp and referee.h to rm_common
* Merge remote-tracking branch 'origin/master'
* Merge branch 'master' into master
* Use “pragma once” in rm_common headers instead of include guards.
* Merge branch 'master' into master
* Contributors: Peter-Chan-tech, QiayuanLiao, chenzheng, qiayuan, ye-luo-xi-tui, yezi

* Format the code
* Format the code
* Add licence
* Merge branch 'rm-controls:master' into master
* Merge pull request `#7 <https://github.com/rm-controls/rm_control/issues/7>`_ from Peter-Chan-tech/master
  Move referee operations to rm_common
* Remove serial lib in rm_common
* Reformat
* Add new line at the end of files
* Move referee.cpp and referee.h to rm_common
* Merge remote-tracking branch 'origin/master'
* Merge branch 'master' into master
* Use “pragma once” in rm_common headers instead of include guards.
* Merge branch 'master' into master
* Contributors: Peter-Chan-tech, QiayuanLiao, chenzheng, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2021-08-14)
------------------
* Run pre-commit
* Add missing CATKIN_DEPENDS in catkin_package()
* Format rm_common using clang-format
* Contributors: qiayuan

0.1.1 (2021-08-12)
------------------
* Reset all version to 0.1.0
* Contributors: qiayuan
