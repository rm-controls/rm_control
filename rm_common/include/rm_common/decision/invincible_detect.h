//
// Created by ray on 25-1-11.
//

#pragma once

#include <ros/ros.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotStatus.h>

namespace rm_common
{
class InvincibleDetect
{
public:
  InvincibleDetect(ros::NodeHandle& nh)
  {
    if (!nh.getParam("invincible_check", invincible_check_))
      ROS_ERROR("Invincible_check no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  typedef enum
  {
    INJURABLE = 0,
    IN_SUPPLY_BASE = 1,
    NORMAL_FRESHLY_RESURRECTED = 2,
    MONEY_FRESHLY_RESURRECTED = 3,
  } InvincibleState;

  //  void updateEnemyStatus(int robot_id)
  // 调用上面的update函数，需要输入敌方id，然后函数中直接更新对应id机器人的无敌状态
  //  InvincibleState getInvincibleState(int robot_id)
  // 用这个函数来get对应机器人的无敌状态，然后用这个状态做判断，如果不是INJURABLE，就把shooter设为READY

private:
  //  updateState()
  void updateRobotInfo(int track_id, rm_msgs::GameRobotStatus& data)
  {
    robot_id = data.robot_id;
    if (robot_id < 100)
      track_id = track_id + 100;
  }

  void updateInvincibleState(int track_id, bool is_red)
  {
  }

  int last_hp;
  float last_revive_time;
  float last_heal_time;
  int robot_id;
  InvincibleState invincible_state = INJURABLE;
  XmlRpc::XmlRpcValue invincible_check_;
};
}  // namespace rm_common
