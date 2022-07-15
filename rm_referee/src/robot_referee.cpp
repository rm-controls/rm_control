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
  add_ui_flag = true;
}

void RobotReferee::drawUi(const ros::Time& time)
{
  RefereeBase::drawUi(time);
  RobotReferee::checkDbusMsg(data_.dbus_data_);
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    if (add_ui_flag)
    {
      trigger_change_ui_->add();
      time_change_ui_->add();
      fixed_ui_->add();
      ROS_INFO("Add ui");
      add_ui_flag = false;
    }
    else
    {
      time_change_ui_->update("capacitor", time);
      if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID && data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
      {
        trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode, false, 1, false);
      }
      else
      {
        trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode,
                                   data_.manual_to_referee_data_.power_limit_state == rm_common::PowerLimit::BURST, 0,
                                   data_.manual_to_referee_data_.power_limit_state == rm_common::PowerLimit::CHARGE);
      }
    }
  }
  else
    add_ui_flag = true;

  flash_ui_->update("spin", time,
                    data_.chassis_cmd_data_.mode == rm_msgs::ChassisCmd::GYRO && data_.vel2d_cmd_data_.angular.z != 0.);
  //  if (data_.dbus_data_.wheel == 0.)
  //    flash_ui_->update("ore", time, data_.dbus_data_.wheel == 0.);
  //  if (data_.dbus_data_.wheel != 0.)
  //    flash_ui_->update("ore", time, data_.dbus_data_.wheel == 0.);

  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);
}
}  // namespace rm_referee
