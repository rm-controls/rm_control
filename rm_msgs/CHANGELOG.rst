^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
