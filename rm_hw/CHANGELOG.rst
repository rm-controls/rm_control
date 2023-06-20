^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_hw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.20 (2023-06-20)
-------------------
* Merge branch 'master' into dev/sentry_map
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/src/referee_base.cpp
* Merge branch 'master' into pitch_ui
* Merge pull request `#160 <https://github.com/ye-luo-xi-tui/rm_control/issues/160>`_ from ye-luo-xi-tui/master
  Filter out actuator data with short time difference
* Filter out actuator data with short time difference.
* Merge branch 'master' into suggest_fire
* Merge branch 'rm-controls:master' into master
* Merge branch 'dev/ui_refact' into dev/balance_cmd_send
* Merge branch 'master' into dev/ui_refact
  # Conflicts:
  #	rm_referee/include/rm_referee/common/protocol.h
  #	rm_referee/src/ui/graph.cpp
* Merge pull request `#146 <https://github.com/ye-luo-xi-tui/rm_control/issues/146>`_ from ye-luo-xi-tui/master
  0.1.19
* Merge branch 'rm-controls:master' into master
* Merge pull request `#1 <https://github.com/ye-luo-xi-tui/rm_control/issues/1>`_ from rm-controls/master
  1
* Contributors: 1moule, ye-luo-xi-tui, yezi, yuchen, 王湘鈜

0.1.19 (2023-05-03)
-------------------
* Merge branch 'master' into multi_dof_controller
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
  #	rm_msgs/CMakeLists.txt
* Merge branch 'master' into one_click_turn_cmd_sender
* Merge branch 'master' into switch_camera_command_sender
  # Conflicts:
  #	rm_referee/include/rm_referee/ui/trigger_change_ui.h
  #	rm_referee/src/ui/trigger_change_ui.cpp
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_control/issues/120>`_ from ye-luo-xi-tui/master
  0.1.18
* Contributors: 1moule, LSY, ye-luo-xi-tui

0.1.18 (2023-03-25)
-------------------
* Merge branch 'master' into dev/polygon_ui
* Merge branch 'master' into gazebo_imu_reserve
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_control/issues/106>`_ from YoujianWu/work
  Delete dependence roslint.
* Run pre-commit.
* Delete dependence roslint.
* Merge branch 'master' into acceleration
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Merge pull request `#104 <https://github.com/ye-luo-xi-tui/rm_control/issues/104>`_ from ye-luo-xi-tui/master
  0.1.17
* Contributors: Kook, ye-luo-xi-tui, yezi, yuchen

0.1.17 (2023-02-21)
-------------------
* Merge branch 'rm-controls:master' into master
* Merge pull request `#84 <https://github.com/ye-luo-xi-tui/rm_control/issues/84>`_ from ye-luo-xi-tui/master
  0.1.16
* Merge branch 'rm-controls:master' into master
* Contributors: ye-luo-xi-tui, 吕骏骐

0.1.16 (2022-11-24)
-------------------
* Merge branch 'master' into new_ui_test
* Merge branch 'master' into dev/command_sender
* Merge pull request `#79 <https://github.com/ye-luo-xi-tui/rm_control/issues/79>`_ from ye-luo-xi-tui/rm_imu_handle
  Add RmImuSensorInterface and add a service to enable or disable imus in rm_gazebo
* Add RmImuSensorInterface.
* Merge branch 'rm-controls:master' into master
* Merge pull request `#70 <https://github.com/ye-luo-xi-tui/rm_control/issues/70>`_ from chenhuiYu00/rm_referee_pr
  Complete the referee part of manual separation.
* Merge branch 'rm-controls:master' into master
* Merge branch 'dev'
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into rm_referee_pr
* Merge pull request `#74 <https://github.com/ye-luo-xi-tui/rm_control/issues/74>`_ from ye-luo-xi-tui/dev
  Update 0.1.15
* Merge pull request `#72 <https://github.com/ye-luo-xi-tui/rm_control/issues/72>`_ from ye-luo-xi-tui/dev
  Fix realtime loop
* Type conversion.
* Merge branch 'master' into rm_referee_pr_buffer
  # Conflicts:
  #	rm_common/include/rm_common/decision/power_limit.h
* Merge pull request `#69 <https://github.com/ye-luo-xi-tui/rm_control/issues/69>`_ from Edwinlinks/gpio-name-modify
  Modify the name of gpio manager which was misnamed before by his developers.
* Merge pull request `#68 <https://github.com/ye-luo-xi-tui/rm_control/issues/68>`_ from Edwinlinks/modify-tof-can
  Add continue which was missing in the previous development.
* Modify the name of gpio manager which was misnamed before.
* Add continue in tof data parsing.
* Merge branch 'master' into referee
  # Conflicts:
  #	rm_common/include/rm_common/decision/service_caller.h
  #	rm_msgs/CMakeLists.txt
  #	rm_msgs/msg/referee/GameRobotStatus.msg
  #	rm_msgs/msg/referee/GameStatus.msg
* Merge branch 'master' into referee1
* Merge branch 'master' into referee1
* Merge branch 'master' into referee1
* Fixed for test manual,Immature work.
* Ljq update,fixed for test manual,Immature work.
* Contributors: Edwinlinks, QiayuanLiao, ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.15 (2022-09-02)
-------------------
* Add namespace.
* Fix realtime loop.
* Contributors: yezi

0.1.14 (2022-06-16)
-------------------

0.1.13 (2022-06-12)
-------------------
* Merge pull request `#58 <https://github.com/rm-controls/rm_control/issues/58>`_ from Edwinlinks/tf-radar-interface
  Update tof radar interface and Add tof radar msg to rm_msgs
* Delete tof sensor interface, modify radar_data to tof_data
* Delete tof sensor interface, TofSensor.msg.
* Change tf_radar_interface to tof_radar_interface and change TfRadarData.msg to TofRadarData.msg
* Add member tf_radar_interface\_ to hardware_interface.h
* Update tf radar interface and Add tf radar msg to rm_msgs
* Contributors: Edwinlinks, ye-luo-xi-tui

0.1.12 (2022-06-11)
-------------------
* Merge pull request `#59 <https://github.com/ye-luo-xi-tui/rm_control/issues/59>`_ from ye-luo-xi-tui/master
  0.1.11
* Contributors: QiayuanLiao

0.1.11 (2022-06-10)
-------------------
* Merge pull request `#52 <https://github.com/ye-luo-xi-tui/rm_control/issues/52>`_ from XYM-github/gpio_interface_2.0
  Changed gpio type in gpio interface to enum.
* Add the namespace of gpio_manager.
* Move gpio type from rm_hw to rm_common.
* Changed gpio type in gpio interface to enum.
* Gpio interface 2 (`#51 <https://github.com/ye-luo-xi-tui/rm_control/issues/51>`_)
  * Write a gpio_manager, it can initialize with pin ID and direction, and write output or read Input.
  * Modifier readInput().
  * Write gpio_state_interface.h(read only)
  * write gpio_state_interface.h(read only)
  * Modifier gpio manager. Write a new writeOutput() function.
  * Add gpio_state_interface in robotHW
  * Solve a error
  * Modifier names of variables.
  * Set GpioReadHandle and GpioWriteHandle. Register GpioReadInterface and GpioWriteInterface. Improve the code about gpio and delete unnecessary things.
  * Add GpioRead.msg GpioWrite.msg.
  * Add Gpio controller.
  * Update Gpio controller.
  * Update Gpio controller.
  * Update gpio controller.
  * Update gpio_controller.
  * Update gpio_controller to version 2.0.
  * Update gpio_interface to version2.0.
  Co-authored-by: yezi <1536117624@qq.com>
  Co-authored-by: ye-luo-xi-tui <74857762+ye-luo-xi-tui@users.noreply.github.com>
  Co-authored-by: QiayuanLiao <liaoqiayuan@gmail.com>
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_control/issues/46>`_ from XYM-github/imu_angular_offset
  Add imu angular_vel_offset.
* Add imu angular_vel_offset.
* Merge remote-tracking branch 'origin/master'
* Contributors: Edwinlinks, QiayuanLiao, XYM-github, Yuexin Mu, YuuinIH, chenzheng, qiayuan, ye-luo-xi-tui, yezi

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
