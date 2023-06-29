^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.20 (2023-06-20)
-------------------
* Merge branch 'master' into dev/power_management
* Fix a judge to adapt 2023 season rule.
* Merge branch 'master' into dev/sentry_map
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/src/referee_base.cpp
* Merge pull request `#161 <https://github.com/ye-luo-xi-tui/rm_control/issues/161>`_ from chenhuiYu00/dev/balance_modechange
  Move loadController() into public.
* Merge branch 'master' into Add_engineerui
* Merge pull request `#163 <https://github.com/ye-luo-xi-tui/rm_control/issues/163>`_ from 1moule/pitch_ui
  Add pitch angle ui for hero
* Add the function of directly setting the target color and add the enumeration of purple.
* Move loadController() into public.
* Merge branch 'master' into pitch_ui
* Merge branch 'master' into pitch_ui
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/include/rm_referee/ui/time_change_ui.h
  #	rm_referee/src/referee_base.cpp
  #	rm_referee/src/ui/time_change_ui.cpp
* Merge pull request `#158 <https://github.com/ye-luo-xi-tui/rm_control/issues/158>`_ from chenhuiYu00/dev/power_management
  Update new power management
* Merge branch 'master' into dev/power_management
* Merge pull request `#159 <https://github.com/ye-luo-xi-tui/rm_control/issues/159>`_ from 1moule/windwill_auto_fire
  Add minimal shoot frequency for hitting buff.
* Modify for format.
* Merge branch 'master' into windwill_auto_fire
* Add minimal shoot frequency for hitting buff.
* Merge pull request `#157 <https://github.com/ye-luo-xi-tui/rm_control/issues/157>`_ from chenhuiYu00/dev/balance_cmd_send
  Add balance command sender and balance pitch ui
* Merge branch 'dev/balance_cmd_send' into dev/power_management
* Delete unused code.
* Merge branch 'dev/power_management' into dev/balance_cmd_send_power_management
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Update super capacitor use strategy
* Delete game_progress\_
* Cancel calibration during 3min preparing
* Adjust is_online judge in power_limit.h
* Merge branch 'master' into dev/power_management
* Merge branch 'master' into dev/ui_refact
* Merge branch 'master' into dev/balance_cmd_send
* Merge pull request `#154 <https://github.com/ye-luo-xi-tui/rm_control/issues/154>`_ from 1moule/suggest_fire
  Modify the shooter command sender to enable hero to automatically hitting rotating armor
* Merge branch 'master' into suggest_fire
* Merge branch 'master' into dev/balance_cmd_send
* Merge branch 'rm-controls:master' into master
* Merge pull request `#148 <https://github.com/ye-luo-xi-tui/rm_control/issues/148>`_ from 1moule/switch_buff
  Add a function to set target type.
* Add a function to set target type.
* Merge branch 'dev/ui_refact' into dev/balance_cmd_send
* Merge branch 'master' into dev/ui_refact
  # Conflicts:
  #	rm_referee/include/rm_referee/common/protocol.h
  #	rm_referee/src/ui/graph.cpp
* Merge pull request `#146 <https://github.com/ye-luo-xi-tui/rm_control/issues/146>`_ from ye-luo-xi-tui/master
  0.1.19
* Cancel referee power limit judge.
* Add safety power select.
* Merge branch 'master' into dev/balance_cmd_send
* Merge branch 'master' into dev/balance_pitch_ui
* Merge branch 'rm-controls:master' into master
* Add chassis cmd send referee_power_limit get.
* Merge branch 'master' into suggest_fire
* Merge branch 'master' into suggest_fire
* Merge branch 'rm-controls:master' into master
* Change vel2D cmd sender's callback function.
* Add balance command sender.
* Modify type of the function "setArmorType".
* Merge branch 'rm-controls:master' into master
* Add armor type to the judgment of checkerror.
* Merge branch 'master' into suggest_fire
* Modify shooter command sender to enable visual control firing.
* Merge pull request `#1 <https://github.com/ye-luo-xi-tui/rm_control/issues/1>`_ from rm-controls/master
  1
* Contributors: 1moule, BruceLannn, ye-luo-xi-tui, yuchen, 王湘鈜

0.1.19 (2023-05-03)
-------------------
* Merge pull request `#142 <https://github.com/ye-luo-xi-tui/rm_control/issues/142>`_ from 1moule/master
  Modify the radio frequency in the burst mode
* Merge branch 'master' into new_protocol
* Merge branch 'rm-controls:master' into master
* Modify the radio frequency in the burst mode.
* Merge pull request `#139 <https://github.com/ye-luo-xi-tui/rm_control/issues/139>`_ from ye-luo-xi-tui/master
  Add safe_speed_limit
* Merge remote-tracking branch 'origin/master'
* Fix format.
* Merge branch 'rm-controls:master' into master
* Add safe_speed_limit.
* Merge pull request `#122 <https://github.com/ye-luo-xi-tui/rm_control/issues/122>`_ from L-SY/multi_dof_controller
  Add MultiDofCommandSender for multi_dof_controller.
* Change function name.
* Change function name.
* Merge pull request `#128 <https://github.com/ye-luo-xi-tui/rm_control/issues/128>`_ from chenhuiYu00/dev/fix_power_limit
  Fixed bug in burst model
* Merge pull request `#132 <https://github.com/ye-luo-xi-tui/rm_control/issues/132>`_ from 1moule/one_click_turn_cmd_sender
  Add a function in gimbal command sender for U-turn
* Merge branch 'master' into one_click_turn_cmd_sender
* Merge branch 'master' into one_click_turn_cmd_sender
* Merge pull request `#127 <https://github.com/ye-luo-xi-tui/rm_control/issues/127>`_ from Aung-xiao/power_limit_pr
  Modify sentry's power limit
* delete old sentry power limit
* Fixed bug in burst model.
* Merge branch 'master' into switch_camera_command_sender
* modify sentry power limit
* Merge pull request `#126 <https://github.com/ye-luo-xi-tui/rm_control/issues/126>`_ from NaHCO3bc/add_fun_in_heat_limit
  Add two function to get data about cooling.
* Merge branch 'master' into multi_dof_controller
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
  #	rm_msgs/CMakeLists.txt
* Merge branch 'master' into one_click_turn_cmd_sender
* Add two function to get data about cooling.
* Merge branch 'master' into switch_camera_command_sender
  # Conflicts:
  #	rm_referee/include/rm_referee/ui/trigger_change_ui.h
  #	rm_referee/src/ui/trigger_change_ui.cpp
* Add multi_dof_commandsender.
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_control/issues/120>`_ from ye-luo-xi-tui/master
  0.1.18
* Added a function in gimbal command sender to set the point of direct mode.
* Add multi_dof_command_sender.
* Remove the modification about command sender on this branch.
* Add ui for switch camera and modefy command sender.
* Add a command sender for switching cameras.
* Contributors: 1moule, Aung-xiao, LSY, NaHCO3bc, ye-luo-xi-tui, yezi, yuchen

0.1.18 (2023-03-25)
-------------------
* Merge pull request `#119 <https://github.com/ye-luo-xi-tui/rm_control/issues/119>`_ from 1moule/fix_power_limit
  Modify the power limit when the capacitor is offline.
* Modify the power limit when the capacitor is offline.
* Merge pull request `#109 <https://github.com/ye-luo-xi-tui/rm_control/issues/109>`_ from ljq-lv/Delete
  Delete the chassis mode "GYRO"
* Merge pull request `#115 <https://github.com/ye-luo-xi-tui/rm_control/issues/115>`_ from 1moule/camera_command_sender
  Add a command serder class for switching camera.
* Delete redundant functions, use the ternary operator to change the data of msg.
* Modify variable name.
* Modify variable name.
* Add a command serder class for switching camera.
* Merge branch 'master' into dev/polygon_ui
* Delete the judge of chassis mode
* Add the param "is_gyro"
* Merge pull request `#110 <https://github.com/ye-luo-xi-tui/rm_control/issues/110>`_ from ye-luo-xi-tui/master
  Add clear() method to Vector3WithFilter
* Merge branch 'acceleration'
* Add clear() method to Vector3WithFilter.
* Delete the chassis mode "GYRO"
* Merge pull request `#108 <https://github.com/ye-luo-xi-tui/rm_control/issues/108>`_ from ye-luo-xi-tui/acceleration
  Remove acceleration computing
* Remove acceleration computing.
* Merge branch 'master' into gazebo_imu_reserve
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_control/issues/106>`_ from YoujianWu/work
  Delete dependence roslint.
* Run pre-commit.
* Delete dependence roslint.
* Merge branch 'master' into acceleration
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Add accel to TrackData.msg and Add a filter.
* Merge pull request `#104 <https://github.com/ye-luo-xi-tui/rm_control/issues/104>`_ from ye-luo-xi-tui/master
  0.1.17
* Publish acceleration.
* Corrected acceleration calculation.
* Contributors: 1moule, Kook, ljq-lv, ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.17 (2023-02-21)
-------------------
* Merge pull request `#95 <https://github.com/ye-luo-xi-tui/rm_control/issues/95>`_ from Edwinlinks/for-location
  Change tf to tf2 in ori_tool for rm_location.
* Change tf to tf2 in ori_tool for rm_location.
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into balance_state
* Merge pull request `#90 <https://github.com/ye-luo-xi-tui/rm_control/issues/90>`_ from chenhuiYu00/dev/commander_sender
  Update gimbal error and track data in the command sender.
* Code style.
* Update shooter command sender.
* Cancel storage of gimbal error data.
* Merge branch 'rm-controls:master' into master
* Merge pull request `#84 <https://github.com/ye-luo-xi-tui/rm_control/issues/84>`_ from ye-luo-xi-tui/master
  0.1.16
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Contributors: Edwinlinks, ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.16 (2022-11-24)
-------------------
* Merge branch 'master' into new_ui_test
* Merge pull request `#78 <https://github.com/ye-luo-xi-tui/rm_control/issues/78>`_ from chenhuiYu00/dev/command_sender
  Check the modification of command sender.
* Cancel passing msg in constructor.
* Cancel constructor transfer cmd.
* Optimize command sender.
* Merge branch 'master' into dev/command_sender
* Delete referee msg and update command sender.
* Merge pull request `#79 <https://github.com/ye-luo-xi-tui/rm_control/issues/79>`_ from ye-luo-xi-tui/rm_imu_handle
  Add RmImuSensorInterface and add a service to enable or disable imus in rm_gazebo
* Add RmImuSensorInterface.
* Merge pull request `#76 <https://github.com/ye-luo-xi-tui/rm_control/issues/76>`_ from chenhuiYu00/accleration_Initial_value
  Add accleration initial value.
* Merge branch 'rm-controls:master' into master
* Add accleration initial value.
* Merge pull request `#70 <https://github.com/ye-luo-xi-tui/rm_control/issues/70>`_ from chenhuiYu00/rm_referee_pr
  Complete the referee part of manual separation.
* Conflict resolution.
* Updata befor merge.
* Merge pull request `#73 <https://github.com/ye-luo-xi-tui/rm_control/issues/73>`_ from 1moule/chassis_power_buffer
  Add power buffer to power limit.
* Modified some variable names.
* Merge pull request `#71 <https://github.com/ye-luo-xi-tui/rm_control/issues/71>`_ from Edwinlinks/dart-command-sender
  Add command sender for dart command sending.
* Merge pull request `#75 <https://github.com/ye-luo-xi-tui/rm_control/issues/75>`_ from ye-luo-xi-tui/master
  Reset imu_filter when robot relive
* Merge branch 'dev'
* Modified some variable names.
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into rm_referee_pr
* Merge pull request `#74 <https://github.com/ye-luo-xi-tui/rm_control/issues/74>`_ from ye-luo-xi-tui/dev
  Update 0.1.15
* Modify prompt information.
* Add power buffer to power limit.
* Add RobotID enum.
* Delete /common/data.h, Update power_limit and heat_limit.
* Update date acquisition in command_sender.
* Move referee part from rm_common to rm_referee and modify ui sending logic.
* Reset imu_filter when robot relive.
* Delete unnecessary offset
* Modify getIndex() function.
* Add command's offset for servo control
* Add ManualToReferee msg.
* Modify joint point command sender for dat manual.
* Merge and fixed conflict.
* Merge branch 'master' into referee
  # Conflicts:
  #	rm_common/include/rm_common/decision/service_caller.h
  #	rm_msgs/CMakeLists.txt
  #	rm_msgs/msg/referee/GameRobotStatus.msg
  #	rm_msgs/msg/referee/GameStatus.msg
* Merge branch 'master' into referee1
* Add defect code.
* Merge branch 'master' into referee1
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
* Fixed for test manual,Immature work.
* Merge branch 'rm_referee1' into referee1
* Ljq update,fixed for test manual,Immature work.
* Update calibration_status_pub's position
* Update game_robot_status, game_status, capacity_data's pub
* Add detection_state pub
* Update stateCommandSender
* Add calibration_status_pub
* Contributors: 1moule, Chenhui, Edwinlinks, QiayuanLiao, ljq-lv, ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.15 (2022-09-02)
-------------------

0.1.14 (2022-06-16)
-------------------
* Merge pull request `#65 <https://github.com/rm-controls/rm_control/issues/65>`_ from ye-luo-xi-tui/param
  Add param to set actual bullet speed
* Modifier variables name.
* Merge branch 'master' into param
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Add params to set actual bullet speed.
* Merge pull request `#63 <https://github.com/rm-controls/rm_control/issues/63>`_ from ye-luo-xi-tui/acceleration
  Don't shoot when target's acceleration is large
* Fix bug.
* Remove a implicit bug and some warning in calibration_queue.h
* Modifier some variables name.
* Put computing acceleration into rm_common.
* Don't shoot when target's acceleration is large,not have moving average.
* Contributors: BruceLannn, QiayuanLiao, qiayuan, yezi

0.1.13 (2022-06-12)
-------------------
* Merge pull request `#58 <https://github.com/rm-controls/rm_control/issues/58>`_ from Edwinlinks/tf-radar-interface
  Update tof radar interface and Add tof radar msg to rm_msgs
* Delete tof sensor interface, TofSensor.msg.
* Change tf_radar_interface to tof_radar_interface and change TfRadarData.msg to TofRadarData.msg
* Update key function and ui.
* Update tf radar interface and Add tf radar msg to rm_msgs
* Contributors: BruceLannn, Edwinlinks, ye-luo-xi-tui

0.1.12 (2022-06-11)
-------------------
* Update logic of changing enemy color.
* Merge pull request `#59 <https://github.com/ye-luo-xi-tui/rm_control/issues/59>`_ from ye-luo-xi-tui/master
  0.1.11
* Contributors: QiayuanLiao, yezi

0.1.11 (2022-06-10)
-------------------
* Add synchronized calibration.
* Changed gpio type in gpio interface to enum.
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
* Merge pull request `#49 <https://github.com/ye-luo-xi-tui/rm_control/issues/49>`_ from ChenZheng29/master
  Delete cost function and modify the track topic
* Delete target_cost_function.cpp and target_cost_function.h
* Merge pull request `#48 <https://github.com/ye-luo-xi-tui/rm_control/issues/48>`_ from ye-luo-xi-tui/master
  Fix a stupid bug
* Fix a stupid bug.
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#47 <https://github.com/ye-luo-xi-tui/rm_control/issues/47>`_ from ye-luo-xi-tui/master
  Decide acceleration and max_vel by power limit
* Decide acceleration and max_vel by power limit.
* Contributors: QiayuanLiao, XYM-github, Yuexin Mu, YuuinIH, chenzheng, qiayuan, yezi

0.1.10 (2022-05-22)
-------------------
* Merge pull request `#42 <https://github.com/rm-controls/rm_control/issues/42>`_ from ye-luo-xi-tui/service
  Add enable_imu_trigger service
* Use publisher instead of real-time publisher.
* Merge pull request `#40 <https://github.com/rm-controls/rm_control/issues/40>`_ from ye-luo-xi-tui/master
  Fix a bug in loading params of imu filter
* Fix a bug in loading param.
* Contributors: QiayuanLiao, yezi

0.1.9 (2022-3-28)
------------------
* Add imu_filter and deprecated imu_extra_handle(Since the update frequency of the control loop is not stable, some of
  the camera trigger signals of imu will be lost. We put the imu filter down to the hardware resource layer, so
  imu_extra_handle is breaking. )
* Add tof sensor interface
* Contributors: Edwinlinks, Jie j, QiayuanLiao, yezi

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
