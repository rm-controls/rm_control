^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.20 (2023-06-20)
-------------------
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
* Contributors: 1moule, ye-luo-xi-tui, yuchen, 王湘鈜

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
* Merge pull request `#107 <https://github.com/ye-luo-xi-tui/rm_control/issues/107>`_ from chenhuiYu00/gazebo_imu_reserve
  Fix imu problem in gazebo simulation.
* Use list instead of vector.
* Merge branch 'master' into gazebo_imu_reserve
* Pre-allocate imu vector memory.
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
* Merge pull request `#99 <https://github.com/ye-luo-xi-tui/rm_control/issues/99>`_ from ye-luo-xi-tui/master
  Fix bug
* Fix bug.
* Add gravity.
* Merge branch 'rm-controls:master' into master
* Merge pull request `#84 <https://github.com/ye-luo-xi-tui/rm_control/issues/84>`_ from ye-luo-xi-tui/master
  0.1.16
* Merge branch 'rm-controls:master' into master
* Contributors: ye-luo-xi-tui, yezi, 吕骏骐

0.1.16 (2022-11-24)
-------------------
* Merge branch 'master' into new_ui_test
* Merge branch 'master' into dev/command_sender
* Merge pull request `#79 <https://github.com/ye-luo-xi-tui/rm_control/issues/79>`_ from ye-luo-xi-tui/rm_imu_handle
  Add RmImuSensorInterface and add a service to enable or disable imus in rm_gazebo
* Add RmImuSensorInterface.
* Add a service to enable or disable imus.
* Merge branch 'dev'
* Merge branch 'rm-controls:master' into master
* Merge branch 'master' into rm_referee_pr
* Merge pull request `#74 <https://github.com/ye-luo-xi-tui/rm_control/issues/74>`_ from ye-luo-xi-tui/dev
  Update 0.1.15
* Merge branch 'master' into referee
  # Conflicts:
  #	rm_common/include/rm_common/decision/service_caller.h
  #	rm_msgs/CMakeLists.txt
  #	rm_msgs/msg/referee/GameRobotStatus.msg
  #	rm_msgs/msg/referee/GameStatus.msg
* Merge branch 'master' into referee1
* Contributors: ye-luo-xi-tui, yezi, yuchen, 吕骏骐

0.1.15 (2022-09-02)
-------------------

0.1.14 (2022-06-16)
-------------------

0.1.13 (2022-06-12)
-------------------

0.1.12 (2022-06-11)
-------------------
* Merge pull request `#59 <https://github.com/ye-luo-xi-tui/rm_control/issues/59>`_ from ye-luo-xi-tui/master
  0.1.11
* Contributors: QiayuanLiao

0.1.11 (2022-06-10)
-------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: QiayuanLiao, YuuinIH, qiayuan

0.1.10 (2022-05-22)
-------------------

0.1.9 (2022-3-28)
------------------
* Revert "Update package.xml"
  This reverts commit 835659c320b5e1219ea9c61d04b4017a0c9a850a.
* Update package.xml
* Merge remote-tracking branch 'origin/master'
* Merge branch 'master' of github.com:ye-luo-xi-tui/rm_control
* Merge pull request `#24 <https://github.com/ye-luo-xi-tui/rm_control/issues/24>`_ from ye-luo-xi-tui/gazebo
  Load imu params into gazebo
* Load imu params into gazebo.
* Contributors: Jie j, QiayuanLiao, YuuinIH, yezi

0.1.8 (2021-12-7)
------------------
* Fix imu inertia and add imu to balance
* Merge branch 'master' into gimbal/opti_or_simplify
* Update CHANGELOG
* Merge branch 'master' into gimbal/opti_or_simplify
* Update URDF of imu
* Contributors: qiayuan

0.1.7 (2021-09-26)
------------------
* 0.1.6
* Update CHANGELOG
* Update URDF of imu
* Merge branch 'namespace' into rm_gazebo/imu_sensor_interface
* Merge pull request `#8 <https://github.com/rm-controls/rm_control/issues/8>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control.
* Change name of namespace:from hardware_interface to rm_control.
* Fix some stupid imu_sensor_interface bug in rm_gazebo
* Tested rm_gazebo imu data using Debug in line.
  TODO: Add gravity and noise to the data
* Add imu_sensor_interface without test.
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.6 (2021-09-26)
------------------
* Update URDF of imu
* Merge branch 'namespace' into rm_gazebo/imu_sensor_interface
* Merge pull request `#8 <https://github.com/rm-controls/rm_control/issues/8>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control.
* Change name of namespace:from hardware_interface to rm_control.
* Fix some stupid imu_sensor_interface bug in rm_gazebo
* Tested rm_gazebo imu data using Debug in line.
  TODO: Add gravity and noise to the data
* Add imu_sensor_interface without test.
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.5 (2021-09-02)
------------------

0.1.4 (2021-09-02)
------------------

0.1.3 (2021-09-01)
------------------
* Merge remote-tracking branch 'origin/master'
* Merge branch 'master' into master
* Use “pragma once” in rm_gazebo headers instead of include guards.
* Merge branch 'master' into master
* Contributors: QiayuanLiao, chenzheng, ye-luo-xi-tui, yezi

* Merge remote-tracking branch 'origin/master'
* Merge branch 'master' into master
* Use “pragma once” in rm_gazebo headers instead of include guards.
* Merge branch 'master' into master
* Contributors: QiayuanLiao, chenzheng, ye-luo-xi-tui, yezi

0.1.2 (2021-08-14)
------------------
* Run pre-commit
* Format rm_gazebo using clang-format
* Contributors: qiayuan

0.1.1 (2021-08-12)
------------------
* Reset all version to 0.1.0
* Contributors: qiayuan
