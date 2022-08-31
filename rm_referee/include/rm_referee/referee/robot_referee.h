//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/referee/referee_base.h"

namespace rm_referee
{
class RobotReferee : public RefereeBase
{
public:
  explicit RobotReferee(ros::NodeHandle& nh, Base& base);
  void robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data_,
                               const ros::Time& last_get_) override;
  void powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_) override;
  void robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_) override;

  void chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data) override;
  void gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data) override;
  void shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data) override;
  void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data) override;

  void addUi() override;

protected:
  TimeChangeUi* time_change_ui_{};
  FlashUi* flash_ui_{};
  TriggerChangeUi* trigger_change_ui_{};
  FixedUi* fixed_ui_{};
};
}  // namespace rm_referee
