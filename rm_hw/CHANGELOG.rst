^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_hw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.10 (2022-05-22)
-------------------
* Merge branch 'rm-controls:master' into master
* Code style
* Merge pull request `#41 <https://github.com/rm-controls/rm_control/issues/41>`_ from Edwinlinks/multi_actuator_transmission
  Update multi_actuator_transmission and delete the double_actuator_tra…
* Merge pull request `#42 <https://github.com/rm-controls/rm_control/issues/42>`_ from ye-luo-xi-tui/service
  Add enable_imu_trigger service
* Initialize structure members in sequence.
* Rename service switch_imu_trigger to enable_imu_trigger.
* Add switch_imu_trigger service.
* Update multi_actuator_transmission and delete the double_actuator_transmission
* Contributors: Edwinlinks, QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

0.1.9 (2022-3-28)
------------------
* Deprecated imu_extra_handle and add imu_filter into hardware resource layer.(Since the update frequency of the control
  loop is not stable, some of the camera trigger signals of imu will be lost. We put the imu filter down to the hardware
  resource layer, so imu_extra_handle is breaking. )
* Merge pull request `#32 <https://github.com/ye-luo-xi-tui/rm_control/issues/32>`_ from Edwinlinks/tof_sensor_interface
  Delete contents in brackets
* Delete contents in brackets
* Merge pull request `#29 <https://github.com/ye-luo-xi-tui/rm_control/issues/29>`_ from Edwinlinks/tof_sensor_interface
  Completed tof_sensor_interface
* Modified the reference order of header files and packet parsing of tof sensor, data type of dis_status
* Add tof sensor interface in rm_common, add parsing can frame in can_bus.cpp, and add TofSensor.msg in rm_msgs.
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#26 <https://github.com/ye-luo-xi-tui/rm_control/issues/26>`_ from ye-luo-xi-tui/master
  Fix a bug in parse imu
* Fix a stupid bug.
* Contributors: Edwinlinks, Jie j, QiayuanLiao, yezi

0.1.8 (2021-12-7)
------------------
* Fix End of files.
* Merge branch 'master' into master
* Update standard4.urdf.xacro and rm_hw/config/standard4.yaml.
* Fix "sorry, unimplemented: non-trivial designated initializers not supported" under melodic
* Merge branch 'master' into gimbal/opti_or_simplify
* Update CHANGELOG
* Set accel_coeff of imu to 6G's
* Receive camera_trigger CAN frame
* Add orientation to ImuExtraHandle
* Update coefficient and standard5.yaml
* Merge branch 'master' into gimbal/opti_or_simplify
* Test can receive of imu2can successfully
* Update CanBus::read() for new imu
* Add ImuExtraInterface
* Contributors: BruceLannn, YuuinIH, qiayuan

0.1.7 (2021-09-26)
------------------
* 0.1.6
* Update CHANGELOG
* Fix some comment messed up by pre-commit
* Merge branch 'namespace' into rm_gazebo/imu_sensor_interface
* Merge pull request `#8 <https://github.com/rm-controls/rm_control/issues/8>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control.
* Change name of namespace:from hardware_interface to rm_control.
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.6 (2021-09-26)
------------------
* Fix some comment messed up by pre-commit
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
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#6 <https://github.com/rm-controls/rm_control/issues/6>`_ from ye-luo-xi-tui/master
  Add doxygen comments on control_loop and transmission
* change doxygen comments on control_loop.
* Merge remote-tracking branch 'origin/master'
* Add doxygen comments on transimission.
* Merge branch 'master' into master
* Add doxygen comments on control_loop.h, double_actuator_transmission.h, double_actuator_transmission_loader.h.
* Use “pragma once” in rm_hw headers instead of include guards.
* Merge branch 'master' into master
* Merge pull request `#4 <https://github.com/rm-controls/rm_control/issues/4>`_ from ye-luo-xi-tui/master
  Add doxygen comments on hardware_interface.h
* Rename rm_base to rm_hw
* update comments on hardware_interface.h
* update comments on hardware_interface.h
* merge
* update comments of hardware_interface.h(not complete)
* Rename RmBaseHardWareInterface to RmRobotHW
* Code style
* Merge pull request `#3 <https://github.com/rm-controls/rm_control/issues/3>`_ from ye-luo-xi-tui/master
  Add doxygen comments on can_bus.h.
* update comments of hardware_interface.h
* update comments of can_bus.h.
* update comments of can_bus.h.
* Merge pull request `#2 <https://github.com/rm-controls/rm_control/issues/2>`_ from ye-luo-xi-tui/master
  Add doxygen comments on socketcan.h
* update comments of functions.
* update comments of functions.
* update comments of functions.
* update comments of functions and fix a spelling error.
* Rename RM_BASE to RM_HW
* update comments of functions
* update comments of functions
* Rename rm_base to rm_hw
* Rename rm_base to rm_hw
* Rename rm_base to rm_hw
* Contributors: BruceLannn, QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

* Format the code
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#6 <https://github.com/rm-controls/rm_control/issues/6>`_ from ye-luo-xi-tui/master
  Add doxygen comments on control_loop and transmission
* change doxygen comments on control_loop.
* Merge remote-tracking branch 'origin/master'
* Add doxygen comments on transimission.
* Merge branch 'master' into master
* Add doxygen comments on control_loop.h, double_actuator_transmission.h, double_actuator_transmission_loader.h.
* Use “pragma once” in rm_hw headers instead of include guards.
* Merge branch 'master' into master
* Merge pull request `#4 <https://github.com/rm-controls/rm_control/issues/4>`_ from ye-luo-xi-tui/master
  Add doxygen comments on hardware_interface.h
* Rename rm_base to rm_hw
* update comments on hardware_interface.h
* update comments on hardware_interface.h
* merge
* update comments of hardware_interface.h(not complete)
* Rename RmBaseHardWareInterface to RmRobotHW
* Code style
* Merge pull request `#3 <https://github.com/rm-controls/rm_control/issues/3>`_ from ye-luo-xi-tui/master
  Add doxygen comments on can_bus.h.
* update comments of hardware_interface.h
* update comments of can_bus.h.
* update comments of can_bus.h.
* Merge pull request `#2 <https://github.com/rm-controls/rm_control/issues/2>`_ from ye-luo-xi-tui/master
  Add doxygen comments on socketcan.h
* update comments of functions.
* update comments of functions.
* update comments of functions.
* update comments of functions and fix a spelling error.
* Rename RM_BASE to RM_HW
* update comments of functions
* update comments of functions
* Rename rm_base to rm_hw
* Rename rm_base to rm_hw
* Rename rm_base to rm_hw
* Contributors: BruceLannn, QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2021-08-14)
------------------
* Run pre-commit
* Fix error: unused variable ‘jnt_config_ok’
* Code style: loadUrdf
* Delete unreachable code
* Format rm_base using clang-format
* Code style
* Contributors: qiayuan

0.1.1 (2021-08-12)
------------------
* Reset all version to 0.1.0
* Contributors: qiayuan
