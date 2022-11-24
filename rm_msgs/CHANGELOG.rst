^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2022-11-24)
-------------------
* Merge pull request `#81 <https://github.com/ye-luo-xi-tui/rm_control/issues/81>`_ from LJT666666/master
  Add "distance_to_image_center" to detection message type
* Add "distance_to_image_center" to detection message type
* Merge branch 'master' into new_ui_test
* Merge pull request `#78 <https://github.com/ye-luo-xi-tui/rm_control/issues/78>`_ from chenhuiYu00/dev/command_sender
  Check the modification of command sender.
* Delete referee msg and update command sender.
* Merge branch 'rm-controls:master' into master
* Merge pull request `#70 <https://github.com/ye-luo-xi-tui/rm_control/issues/70>`_ from chenhuiYu00/rm_referee_pr
  Complete the referee part of manual separation.
* Merge branch 'dev'
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into rm_referee_pr
* Merge pull request `#74 <https://github.com/ye-luo-xi-tui/rm_control/issues/74>`_ from ye-luo-xi-tui/dev
  Update 0.1.15
* Add RobotID enum.
* Delete /common/data.h, Update power_limit and heat_limit.
* Move referee part from rm_common to rm_referee and modify ui sending logic.
* Add referee is_online msg.
* Add ManualToReferee msg.
* Add referee msg.
* Merge and fixed conflict.
* Merge branch 'master' into referee
  # Conflicts:
  #	rm_common/include/rm_common/decision/service_caller.h
  #	rm_msgs/CMakeLists.txt
  #	rm_msgs/msg/referee/GameRobotStatus.msg
  #	rm_msgs/msg/referee/GameStatus.msg
* Merge branch 'master' into referee1
* Add radar part.
* Merge branch 'master' into referee1
* Ui work success,ore ui is in test.
* Merge branch 'referee1' of github.com:chenhuiYu00/rm_control into referee1
   Conflicts:
  	rm_common/include/rm_common/decision/command_sender.h
  	rm_common/include/rm_common/decision/service_caller.h
  	rm_msgs/msg/referee/CalibrationStatus.msg
  	rm_msgs/msg/referee/CapacityData.msg
  	rm_msgs/msg/referee/DetectionStatus.msg
  	rm_msgs/msg/referee/EngineerCmd.msg
  	rm_msgs/msg/referee/GameRobotHp.msg
  	rm_msgs/msg/referee/GameRobotStatus.msg
  	rm_msgs/msg/referee/GameStatus.msg
  	rm_msgs/msg/referee/PowerHeatData.msg
  	rm_msgs/msg/referee/StateCmd.msg
  	rm_referee/config/standard3.yaml
  	rm_referee/include/rm_referee/common/data.h
  	rm_referee/include/rm_referee/common/referee_base.h
  	rm_referee/include/rm_referee/engineer_referee.h
  	rm_referee/include/rm_referee/hero_referee.h
  	rm_referee/include/rm_referee/referee/referee.h
  	rm_referee/include/rm_referee/referee/ui.h
  	rm_referee/include/rm_referee/robot_referee.h
  	rm_referee/include/rm_referee/standard_referee.h
  	rm_referee/launch/load.launch
  	rm_referee/src/common/referee_base.cpp
  	rm_referee/src/engineer_referee.cpp
  	rm_referee/src/hero_referee.cpp
  	rm_referee/src/main.cpp
  	rm_referee/src/referee/referee.cpp
  	rm_referee/src/referee/ui.cpp
  	rm_referee/src/robot_referee.cpp
  	rm_referee/src/standard_referee.cpp
* Merge branch 'master' into referee1
* Fixed topic naming, add time stamp in referee msgs.
* Fixed for test manual,Immature work.
* Merge branch 'rm_referee1' into referee1
* Ljq update,fixed for test manual,Immature work.
* Add PowerHearData.msg and GameRObotHp.msg
* Add related msg of referee
* Add related msg of referee
* Add related msg of referee
* Fixed the wrong type
* Contributors: Chenhui, LJT666666, QiayuanLiao, ljq-lv, ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.15 (2022-09-02)
-------------------

0.1.14 (2022-06-16)
-------------------
* Merge branch 'master' into param
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Merge pull request `#64 <https://github.com/rm-controls/rm_control/issues/64>`_ from Edwinlinks/referee-msgs
  Add msgs of rm_referee
* Delete the unnecessary msgs from CMakeLists.txt
* Delete the unnecessary msg
* Modify the code style error of referee msg
* Add msgs of rm_referee
* Contributors: Edwinlinks, QiayuanLiao, yezi

0.1.13 (2022-06-12)
-------------------
* Merge pull request `#58 <https://github.com/rm-controls/rm_control/issues/58>`_ from Edwinlinks/tf-radar-interface
  Update tof radar interface and Add tof radar msg to rm_msgs
* Delete tof sensor interface, TofSensor.msg.
* Change tf_radar_interface to tof_radar_interface and change TfRadarData.msg to TofRadarData.msg
* Add TfRadarData.msg to add_message_files
* Update tf radar interface and Add tf radar msg to rm_msgs
* Contributors: Edwinlinks, ye-luo-xi-tui

0.1.12 (2022-06-11)
-------------------
* Merge pull request `#59 <https://github.com/ye-luo-xi-tui/rm_control/issues/59>`_ from ye-luo-xi-tui/master
  0.1.11
* Contributors: QiayuanLiao

0.1.11 (2022-06-10)
-------------------
* Merge pull request `#55 <https://github.com/ye-luo-xi-tui/rm_control/issues/55>`_ from jceleven/master
  Add the use_id_classification flag.
* Add the use_id_classification flag.
* Merge pull request `#53 <https://github.com/ye-luo-xi-tui/rm_control/issues/53>`_ from ye-luo-xi-tui/master
  Delete target_velocity in GimbalCmd.msg
* Merge pull request `#54 <https://github.com/ye-luo-xi-tui/rm_control/issues/54>`_ from ye-luo-xi-tui/command_source_frame
  Add command_source_frame to ChassisCmd.msg
* Add command_source_frame to ChassisCmd.msg.
* Delete target_velocity in GimbalCmd.msg.
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
* Merge pull request `#49 <https://github.com/ye-luo-xi-tui/rm_control/issues/49>`_ from ChenZheng29/master
  Delete cost function and modify the track topic
* Modify the track topic name and message, and unify the track interface
* Merge remote-tracking branch 'origin/master'
* Add testing option to shooter for testing the trigger without friction wheel
* Contributors: Jiachen Shen, QiayuanLiao, Yuexin Mu, YuuinIH, chenzheng, qiayuan, yezi

0.1.10 (2022-05-22)
-------------------
* Merge pull request `#43 <https://github.com/rm-controls/rm_control/issues/43>`_ from ye-luo-xi-tui/track_msg
  Add TrackCmd.msg
* Modifier TrackCmd.msg format.
* Add TrackCmd.msg.
* Merge pull request `#42 <https://github.com/rm-controls/rm_control/issues/42>`_ from ye-luo-xi-tui/service
  Add enable_imu_trigger service
* Rename ImuTriggerSwitch.srv to EnableImuTrigger.srv and add something.
* Add ImuTriggerSwitch.srv.
* Contributors: QiayuanLiao, yezi

0.1.9 (2022-3-28)
------------------
* Update rm_msgs cmake minimum required. (`#36 <https://github.com/ye-luo-xi-tui/rm_control/issues/36>`_)
* Merge pull request `#27 <https://github.com/ye-luo-xi-tui/rm_control/issues/27>`_ from Zhouzhenjie/master
  Add the service for the conversation between a camera and a imu.
* Merge pull request `#29 <https://github.com/ye-luo-xi-tui/rm_control/issues/29>`_ from Edwinlinks/tof_sensor_interface
  Completed tof_sensor_interface
* Modified the reference order of header files and packet parsing of tof sensor, data type of dis_status
* Add tof sensor interface in rm_common, add parsing can frame in can_bus.cpp, and add TofSensor.msg in rm_msgs.
* Merge remote-tracking branch 'origin/master'
* Add the service for the conversation between a camera and a imu.
* Contributors: Edwinlinks, Jie j, QiayuanLiao, YuuinIH

0.1.8 (2021-12-7)
------------------
* Merge branch 'master' into gimbal/opti_or_simplify
* Update CHANGELOG
* Remove cover of ShooterCmd
* Contributors: qiayuan

0.1.7 (2021-09-26)
------------------
* 0.1.6
* Update CHANGELOG
* Merge branch 'gimbal/opti_or_simplify'
* Modified GimbalCmd.msg, and delete moving_average_filter
* Contributors: qiayuan

0.1.6 (2021-09-26)
------------------
* Merge branch 'gimbal/opti_or_simplify'
* Modified GimbalCmd.msg, and delete moving_average_filter
* Contributors: qiayuan

0.1.5 (2021-09-02)
------------------

0.1.4 (2021-09-02)
------------------

0.1.3 (2021-09-01)
------------------
* Merge branch 'master' into master
* Rename rm_base to rm_hw
* Contributors: QiayuanLiao, qiayuan

* Merge branch 'master' into master
* Rename rm_base to rm_hw
* Contributors: QiayuanLiao, qiayuan

0.1.2 (2021-08-14)
------------------
* Run pre-commit
* Add missing CATKIN_DEPENDS in catkin_package()
* Contributors: qiayuan

0.1.1 (2021-08-12)
------------------
* Reset all version to 0.1.0
* Contributors: qiayuan
