^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
