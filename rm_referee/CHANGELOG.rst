^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_referee
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
