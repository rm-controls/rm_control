//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/robot_referee.h"

namespace rm_referee
{
RobotReferee::RobotReferee(ros::NodeHandle& nh) : RefereeBase(nh)
{
  ros::NodeHandle ui_nh(nh, "ui");
  trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
  time_change_ui_ = new TimeChangeUi(ui_nh, data_);
  flash_ui_ = new FlashUi(ui_nh, data_);
  fixed_ui_ = new FixedUi(ui_nh, data_);
}

void RobotReferee::getPowerLimitStatus(double limit_power_, int referee_power_limit)
{
  if (limit_power_ == 0)
    power_limit_state = rm_common::PowerLimit::TEST;
  else if (limit_power_ == (double)referee_power_limit || (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_z))
    power_limit_state = rm_common::PowerLimit::NORMAL;
  else if (limit_power_ == (double)referee_power_limit * 0.85 || data_.dbus_data_.key_b)
    power_limit_state = rm_common::PowerLimit::CHARGE;
  else if (data_.referee_.referee_data_.capacity_data.cap_power_ > capacitor_threshold || data_.dbus_data_.key_shift)
  {
    if (data_.chassis_cmd_data_.mode == rm_msgs::ChassisCmd::GYRO || limit_power_ == referee_power_limit + extra_power)
      power_limit_state = rm_common::PowerLimit::BURST;
    else if (limit_power_ == burst_power ||
             limit_power_ == data_.referee_.referee_data_.game_robot_status_.chassis_power_limit_)
      power_limit_state = rm_common::PowerLimit::BURST;
  }
  else
    ROS_INFO("Not get power limit status,ignorable if Infrequently.");

  ROS_INFO("Power limit state: %d", power_limit_state);
}

void RobotReferee::drawUi(const ros::Time& time)
{
  RefereeBase::drawUi(time);
  RobotReferee::checkDbusMsg(data_.dbus_data_);
  getPowerLimitStatus(data_.referee_.referee_data_.capacity_data.limit_power_,
                      data_.referee_.referee_data_.game_robot_status_.chassis_power_limit_);
  time_change_ui_->update("capacitor", time);
  time_change_ui_->update("ore", time);
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID && data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    //    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::TEST);
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode, false, 1, false);
  }
  else
  {
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode,
                               power_limit_state == rm_common::PowerLimit::BURST, 0,
                               power_limit_state == rm_common::PowerLimit::CHARGE);
  }
  // flash_ui_->update("spin", time,
  // data_.chassis_cmd_data_.mode == rm_msgs::ChassisCmd::GYRO && data_.vel2d_cmd_data_.angular.z != 0.);
  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);

  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    trigger_change_ui_->add();
    time_change_ui_->add();
    fixed_ui_->add();
  }
}
}  // namespace rm_referee
