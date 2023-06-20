^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_referee
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.20 (2023-06-20)
-------------------
* Merge pull request `#170 <https://github.com/ye-luo-xi-tui/rm_control/issues/170>`_ from chenhuiYu00/dev/power_management
  Add other unpack code for power management
* Update power management unpack code.
* Merge pull request `#169 <https://github.com/ye-luo-xi-tui/rm_control/issues/169>`_ from chenhuiYu00/dev/update_chassis_topic
  Update chassis topic.
* Update chassis topic.
* Merge pull request `#168 <https://github.com/ye-luo-xi-tui/rm_control/issues/168>`_ from chenhuiYu00/dev/update_power_management
  Update PowerManagement protocol v1.2
* Update PowerManagement protocol v1.2
* Merge pull request `#167 <https://github.com/ye-luo-xi-tui/rm_control/issues/167>`_ from chenhuiYu00/dev/radar_interactive
  Add radar interactive
* Merge pull request `#166 <https://github.com/ye-luo-xi-tui/rm_control/issues/166>`_ from chenhuiYu00/dev/sentry_map
  Add sentry map
* Add radar interactive.
* Merge branch 'master' into dev/sentry_map
  # Conflicts:
  #	rm_referee/src/referee_base.cpp
* Merge branch 'master' into dev/sentry_map
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/src/referee_base.cpp
* Fixed cycle number.
* Merge pull request `#165 <https://github.com/ye-luo-xi-tui/rm_control/issues/165>`_ from rm-controls/revert-164-Add_engineerui
  Revert "Merge the engineer's ui into master"
* Revert "Merge the engineer's ui into master"
* Merge pull request `#164 <https://github.com/ye-luo-xi-tui/rm_control/issues/164>`_ from ljq-lv/Add_engineerui
  Merge the engineer's ui into master
* Merge branch 'master' into Add_engineerui
* Merge pull request `#163 <https://github.com/ye-luo-xi-tui/rm_control/issues/163>`_ from 1moule/pitch_ui
  Add pitch angle ui for hero
* Merge the engineer's ui into master
* Merge branch 'master' into pitch_ui
* Merge branch 'master' into pitch_ui
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/include/rm_referee/ui/time_change_ui.h
  #	rm_referee/src/referee_base.cpp
  #	rm_referee/src/ui/time_change_ui.cpp
* Add pitch angle ui.
* Merge pull request `#158 <https://github.com/ye-luo-xi-tui/rm_control/issues/158>`_ from chenhuiYu00/dev/power_management
  Update new power management
* Merge branch 'master' into windwill_auto_fire
* Change ros info during unpack.
* Merge pull request `#157 <https://github.com/ye-luo-xi-tui/rm_control/issues/157>`_ from chenhuiYu00/dev/balance_cmd_send
  Add balance command sender and balance pitch ui
* Add capacity mode check mode function.
* Add map sentry interactive data send.
* Merge branch 'dev/balance_cmd_send' into dev/power_management
  # Conflicts:
  #	rm_referee/src/referee_base.cpp
* Parameterize add ui and send ui config.
* Reduce add ui frequency.
* Update NORMAL delay.
* Fixed foolish delay bug.
* Merge branch 'dev/power_management' into dev/balance_cmd_send_power_management
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Merge remote-tracking branch 'chenhuiYu/dev/power_management' into dev/power_management
* Update super capacitor use strategy
* Allow chassis ui update during add ui.
* Reduce update ui frequency.
* Merge branch 'master' into dev/balance_cmd_send
* Update balance pitch ui for new ui logic.
* Merge pull request `#156 <https://github.com/ye-luo-xi-tui/rm_control/issues/156>`_ from chenhuiYu00/ui_refact
  Fixed bug in graph queue update
* Merge branch 'master' into dev/balance_cmd_send
  # Conflicts:
  #	rm_referee/src/referee_base.cpp
  #	rm_referee/src/ui/time_change_ui.cpp
* Merge pull request `#145 <https://github.com/ye-luo-xi-tui/rm_control/issues/145>`_ from chenhuiYu00/ui_refact
  Refact ui send logic and add rotation，target state ui
* Merge branch 'dev/ui_refact' into dev/power_management
* Fixed queue bug.
* Delete unused function.
* Merge branch 'dev/ui_refact' into dev/power_management
  # Conflicts:
  #	rm_referee/include/rm_referee/ui/time_change_ui.h
  #	rm_referee/src/ui/time_change_ui.cpp
* Add exit BURST delay.
* Fixed unpack bug.
* Delete old capacity protocol publisher and related code.
* Fixed ui send bug.
* Merge branch 'master' into dev/power_management
* Rename TriggerScalerTriggerChangeUi.
* Merge branch 'master' into dev/ui_refact
* Add power management sample and state data publish.
* Update power management protocol ver1.1
* Fixed getRobotInfo();
* Merge branch 'master' into suggest_fire
* Merge branch 'master' into dev/balance_cmd_send
* Merge branch 'rm-controls:master' into master
* Merge pull request `#150 <https://github.com/ye-luo-xi-tui/rm_control/issues/150>`_ from 1moule/buff_ui
  Update big buff and small buff ui.
* Update big buff and small buff ui.
* Add graph queue.
* Fixed unpack problem in capacity serial .
* Merge branch 'dev/ui_refact' into dev/balance_cmd_send
* Add debug info.
* Update power management protocol.
* Fixed merge problem.
* Merge branch 'master' into dev/ui_refact
  # Conflicts:
  #	rm_referee/include/rm_referee/common/protocol.h
  #	rm_referee/src/ui/graph.cpp
* Add balance pitch ui.
* Merge pull request `#146 <https://github.com/ye-luo-xi-tui/rm_control/issues/146>`_ from ye-luo-xi-tui/master
  0.1.19
* Merge branch 'dev/ui_refact' into dev/balance_cmd_send
  # Conflicts:
  #	rm_referee/include/rm_referee/referee_base.h
  #	rm_referee/include/rm_referee/ui/time_change_ui.h
  #	rm_referee/include/rm_referee/ui/ui_base.h
  #	rm_referee/src/referee_base.cpp
  #	rm_referee/src/ui/ui_base.cpp
* Change ui circle add.
* Add target scale trigger change ui.
* Add rotation time change ui.
* Fixed bugs in referee refactoring.
* Refactoring referee.
* Merge pull request `#144 <https://github.com/ye-luo-xi-tui/rm_control/issues/144>`_ from ye-luo-xi-tui/master
  Fix problem generate by merge conflict
* Add balance pitch ui.
* Merge branch 'master' into dev/balance_cmd_send
* Merge branch 'master' into suggest_fire
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into suggest_fire
* Merge pull request `#1 <https://github.com/ye-luo-xi-tui/rm_control/issues/1>`_ from rm-controls/master
  1
* Contributors: 1moule, BruceLannn, ljq-lv, ye-luo-xi-tui, yuchen, 王湘鈜

0.1.19 (2023-05-03)
-------------------
* Fix problem generate by merge conflict.
* Merge pull request `#143 <https://github.com/ye-luo-xi-tui/rm_control/issues/143>`_ from rm-controls/new_protocol
  Merge branch 'new protocol' into 'master'
* Merge branch 'master' into new_protocol
* Merge branch 'rm-controls:master' into master
* Merge pull request `#141 <https://github.com/ye-luo-xi-tui/rm_control/issues/141>`_ from Aung-xiao/master
  Publish hp msgs in game_robot_state topic
* Merge pull request `#138 <https://github.com/ye-luo-xi-tui/rm_control/issues/138>`_ from chenhuiYu00/fix_ui_param
  Fixed the issue that referee must set ui param.
* add hp msgs
* Fixed ui param get.
* Merge pull request `#134 <https://github.com/ye-luo-xi-tui/rm_control/issues/134>`_ from NaHCO3bc/new_protocol
  Modify the publisher name.
* Modify the publisher name.
* Merge pull request `#131 <https://github.com/ye-luo-xi-tui/rm_control/issues/131>`_ from NaHCO3bc/new_protocol
  Add a publisher to send target position from serial.
* Merge branch 'master' into one_click_turn_cmd_sender
* Add a publisher to send target position from serial.
* Merge pull request `#130 <https://github.com/ye-luo-xi-tui/rm_control/issues/130>`_ from chenhuiYu00/dev/serial_update
  Update server protocl
* Update code style in protocol struct.
* Merge pull request `#125 <https://github.com/ye-luo-xi-tui/rm_control/issues/125>`_ from chenhuiYu00/dev/circle_add
  Add circle in ui add.
* Update serial protocol.
* Merge pull request `#114 <https://github.com/ye-luo-xi-tui/rm_control/issues/114>`_ from 1moule/switch_camera_command_sender
  Add ui for switching cameras
* Modify the logic of getting the camera name parameter.
* Merge branch 'master' into switch_camera_command_sender
* Update official serial protocol.
* Delete unused value.
* Merge branch 'master' into dev/circle_add
* Raise add frequence.
* Merge pull request `#123 <https://github.com/ye-luo-xi-tui/rm_control/issues/123>`_ from ljq-lv/modify
  Modified the msg of engineer  to 'EngineerUi'
* Merge branch 'master' into dev/circle_add
* Use timer in ui add.
* Merge branch 'master' into multi_dof_controller
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
  #	rm_msgs/CMakeLists.txt
* Merge pull request `#124 <https://github.com/ye-luo-xi-tui/rm_control/issues/124>`_ from chenhuiYu00/delete_armor
  Delete armor flash ui.
* Delete armor flash ui.
* Add circle in ui add.
* Merge branch 'master' into one_click_turn_cmd_sender
* Modify the logic of getting the camera name from the configuration file.
* Modified the msg of engineer  to 'EngineerUi'
* Merge branch 'master' into switch_camera_command_sender
  # Conflicts:
  #	rm_referee/include/rm_referee/ui/trigger_change_ui.h
  #	rm_referee/src/ui/trigger_change_ui.cpp
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_control/issues/120>`_ from ye-luo-xi-tui/master
  0.1.18
* Modified to get the camera name from the configuration file.
* Calling display function in updateCameraName function and modefy function parameters of setColor function.
* Modefy ui name and add updateConfig function.
* Add ui for switch camera and modefy command sender.
* Contributors: 1moule, Aung-xiao, LSY, NaHCO3bc, ljq-lv, ye-luo-xi-tui, yezi, yuchen

0.1.18 (2023-03-25)
-------------------
* Merge pull request `#113 <https://github.com/ye-luo-xi-tui/rm_control/issues/113>`_ from chenhuiYu00/dev/polygon_ui
  Add polygon UI.
* Merge pull request `#109 <https://github.com/ye-luo-xi-tui/rm_control/issues/109>`_ from ljq-lv/Delete
  Delete the chassis mode "GYRO"
* Merge pull request `#116 <https://github.com/ye-luo-xi-tui/rm_control/issues/116>`_ from ye-luo-xi-tui/master
  Display shooter block state
* Delete ShootCmd subscriber.
* Display shooter block state.
* Delete test polygon ui.
* Merge branch 'master' into dev/polygon_ui
* Delete the chassis mode "GYRO"
* Merge branch 'master' into gazebo_imu_reserve
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_control/issues/106>`_ from YoujianWu/work
  Delete dependence roslint.
* Run pre-commit.
* Delete dependence roslint.
* Test GroupUi in real robot.
* Rebase FixedUi.
* Add GroupUiBase class and Polygon ui.
* Merge branch 'master' into acceleration
  # Conflicts:
  #	rm_common/include/rm_common/decision/command_sender.h
* Merge pull request `#104 <https://github.com/ye-luo-xi-tui/rm_control/issues/104>`_ from ye-luo-xi-tui/master
  0.1.17
* Contributors: Kook, ljq-lv, ye-luo-xi-tui, yezi, yuchen

0.1.17 (2023-02-21)
-------------------
* Merge pull request `#100 <https://github.com/ye-luo-xi-tui/rm_control/issues/100>`_ from ljq-lv/modified_msgs_name
  Modified msgs name
* Modified the function and topic name
* Modified the function and topic name
* Modified the msg "EngineerCmd" name
* Merge pull request `#88 <https://github.com/ye-luo-xi-tui/rm_control/issues/88>`_ from sloretz/sloretz_remove_invalid_char
  Remove 0x01 char from changelog
* Merge pull request `#86 <https://github.com/ye-luo-xi-tui/rm_control/issues/86>`_ from chenhuiYu00/dev/lane_line_ui
  Add lane line ui.
* Add reference_joint param.
* Remove 0x01 char from changelog
* Add pitch index.
* Update Ui.
* Optimize LaneLine ui.
* Merge branch 'rm-controls:master' into master
* Add LaneLine ui.
* Merge pull request `#84 <https://github.com/ye-luo-xi-tui/rm_control/issues/84>`_ from ye-luo-xi-tui/master
  0.1.16
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Contributors: Shane Loretz, ljq-lv, ye-luo-xi-tui, yuchen, 吕骏骐

0.1.16 (2022-11-24)
-------------------
* Merge pull request `#80 <https://github.com/ye-luo-xi-tui/rm_control/issues/80>`_ from ljq-lv/new_ui_test
  Improve the Ui to reduce data transport
* Modified the braces of rm_common
* Modified the braces of rm_common
* Fixed the error of wrong named
* Fixed the error of wrong named
* Modified the name of time stamp
* Delete the director of "referee" in CMakeLists.txt
* Improve the struct of directory
* Divide ui.cpp into different type cpp
* Add the empty function to updateManualCmdData()
* Move updateManualCmdData into parent class
* Delete the usleep()
* Add the rpc_value of fixed
* Delete unnecessary function run()
* Modified the param's name and combined the if
* Modified the named of time
* Add destructor function
* Modified the logic to get param
* Modified the named of time stamp
* Improve the way to get param
* Delete unnecessary init and NodeHandle
* Move the class's init from cpp to h
* Move referee_ui\_'s init into rm_referee::Referee
* Delete the parent Delete the part of update
* Delete the part of Referee.msg
* Run pre-commit
* Merge branch 'master' into new_ui_test
* Merge pull request `#78 <https://github.com/ye-luo-xi-tui/rm_control/issues/78>`_ from chenhuiYu00/dev/command_sender
  Check the modification of command sender.
* Add namespace "referee" before topic's name
* Fixed code style
* Add "referee\_" before topic's name
* Fixed the spelling error
* Fixed the spelling error
* Fixed the cover state's bug
* Test the basic ui function successful
* Locate the bug of capacity class
* Locate the bug of chassis class
* Add the chassis class
* Fix the logic of CheckUiAdd()
* Delete chassis class to test bug
* Modified the struct of ui
* Modified the struct of ui
* Test the init of referee_base
* Add test code
* Delete referee msg and update command sender.
* test
* Move the robot_id and robot_color from "Base" to the class "DataTranslation"
* Add the class "DataTranslation" to deal with serial\_
* Merge pull request `#76 <https://github.com/ye-luo-xi-tui/rm_control/issues/76>`_ from chenhuiYu00/accleration_Initial_value
  Add accleration initial value.
* Merge branch 'rm-controls:master' into master
* Code format.
* Merge pull request `#70 <https://github.com/ye-luo-xi-tui/rm_control/issues/70>`_ from chenhuiYu00/rm_referee_pr
  Complete the referee part of manual separation.
* Remove referee config.
* Move files.
* Type conversion.
* Add RobotID enum.
* Delete /common/data.h, Update power_limit and heat_limit.
* Update date acquisition in command_sender.
* Naming conventions.
* Move referee part from rm_common to rm_referee and modify ui sending logic.
* Add referee is_online msg.
* Adjust referee data sending way and adapt current ui.
* Add referee msg.
* Merge and fixed conflict.
* Write radar interactive.
* Add radar part.
* Fixed bug.
* Add gimablchassis ui.
* Ui work success,ore ui is in test.
* Try reuse power limit state.
* Change Variable name,color problem in powerlimitstate.
* Fixed some problems,not ready.
* Update config,referee only send ui once.
* Add ore remain,dart remain ui.
* Fixed topic naming, add time stamp in referee msgs.
* Merge date.
* Fixed for test manual,Immature work.
* Merge branch 'rm_referee1' into referee1
* Merge date.
* Ljq update,fixed for test manual,Immature work.
* Add PowerHearData.msg and GameRObotHp.msg
* Delete unnecessary calibraiton.h
* Add README.md
* Update the add operation of ui
* Add referee package
* Contributors: Chenhui, QiayuanLiao, ljq-lv, ye-luo-xi-tui, yuchen, 吕骏骐
