//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/common/referee_base.h"
#include "time.h"

namespace rm_referee
{
class RobotReferee : public RefereeBase
{
public:
  explicit RobotReferee(ros::NodeHandle& nh);

protected:
  void drawUi(const ros::Time& time) override;

  void checkDbusMsg(const rm_msgs::DbusData& dbus_data)
  {
    // eject
    if (dbus_data.key_ctrl && dbus_data.key_r)
    {
      if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO ||
           data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO))
      {
        gimbal_eject = true;
      }
    }
    else if (dbus_data.key_w || dbus_data.key_s || dbus_data.key_a || dbus_data.key_d)
    {
      if ((data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO ||
           data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO) &&
          gimbal_eject)
      {
        chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
        gimbal_eject = false;
      }
    }

    // chassis mode
    if (std::abs(data_.dbus_data_.wheel) > 0.01)
    {
      chassis_mode = rm_msgs::ChassisCmd::GYRO;
    }
    else if (dbus_data.s_r == rm_msgs::DbusData::UP || dbus_data.s_r == rm_msgs::DbusData::DOWN ||
             dbus_data.s_r == rm_msgs::DbusData::MID)
    {
      chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
    }
    else if (dbus_data.key_ctrl && dbus_data.key_z)
    {
      chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
    }
    else if (dbus_data.key_shift)
    {
      chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
    }
    else if (dbus_data.key_g)
    {
      if (chassis_mode == rm_msgs::ChassisCmd::GYRO)
        chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
      else
        chassis_mode = rm_msgs::ChassisCmd::GYRO;
    }
    else if (dbus_data.key_e)
    {
      if (chassis_mode == rm_msgs::ChassisCmd::TWIST)
        chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
      else
        chassis_mode = rm_msgs::ChassisCmd::TWIST;
    }
    else if (dbus_data.key_ctrl && dbus_data.key_r)  // it will cal service
    {
      if (data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO ||
          data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_HERO)
      {
        chassis_mode = rm_msgs::ChassisCmd::GYRO;
      }
    }
  }

  bool gimbal_eject, add_ui_flag;
  int chassis_mode;
  TimeChangeUi* time_change_ui_{};
  FlashUi* flash_ui_{};
  TriggerChangeUi* trigger_change_ui_{};
  FixedUi* fixed_ui_{};

private:
  double capacitor_threshold, extra_power, burst_power;
};
}  // namespace rm_referee
