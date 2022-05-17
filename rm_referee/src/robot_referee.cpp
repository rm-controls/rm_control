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

void RobotReferee::drawUi(const ros::Time& time)
{
  RefereeBase::drawUi(time);
  time_change_ui_->update("capacitor", time);
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID && data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    //    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::TEST);
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode, false, 1, false);
  }
  else
  {
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode,
                               data_.chassis_cmd_data_.power_limit_state == rm_common::PowerLimit::BURST, 0,
                               data_.chassis_cmd_data_.power_limit_state == rm_common::PowerLimit::CHARGE);
  }
  flash_ui_->update("spin", time,
                    data_.chassis_cmd_data_.power_limit_state == rm_msgs::ChassisCmd::GYRO &&
                        data_.vel2d_cmd_data_.angular.z != 0.);
  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);
}
}  // namespace rm_referee