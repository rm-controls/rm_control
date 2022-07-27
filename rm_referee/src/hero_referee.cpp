//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/hero_referee.h"

namespace rm_referee
{
HeroReferee::HeroReferee(ros::NodeHandle& nh, Data& data) : RobotReferee(nh, data)
{
  HeroReferee::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &HeroReferee::gimbalCmdDataCallback, this);
  HeroReferee::shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                                &HeroReferee::shootCmdDataCallback, this);
  HeroReferee::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &HeroReferee::manualDataCallBack, this);
}

void HeroReferee::capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, const ros::Time& last_get_)
{
  RefereeBase::capacityDataCallBack(capacity_data_, last_get_);
  time_change_ui_->update("capacitor", last_get_);

  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_shift && data_.dbus_data_.key_b)
  {
    trigger_change_ui_->update("chassis", 254, 0);
  }
}

void HeroReferee::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  RobotReferee::gimbalCmdDataCallback(data);
  trigger_change_ui_->update("gimbal", data_.gimbal_cmd_data_.mode, data_.manual_to_referee_data_.gimbal_eject);
}

void HeroReferee::shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
{
  RobotReferee::shootCmdDataCallback(data);
  trigger_change_ui_->update("shooter", data_.shoot_cmd_data_.mode, 0, data_.manual_to_referee_data_.shoot_frequency,
                             false);

  if (data_.base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && data_.base_.robot_id_ != rm_referee::RobotId::RED_HERO)
    trigger_change_ui_->update("target", data_.manual_to_referee_data_.det_target,
                               data_.manual_to_referee_data_.shoot_frequency == rm_common::HeatLimit::BURST,
                               data_.manual_to_referee_data_.det_armor_target,
                               data_.manual_to_referee_data_.det_color == rm_msgs::StatusChangeRequest::RED);
  else
    trigger_change_ui_->update("target", data_.manual_to_referee_data_.gimbal_eject,
                               data_.manual_to_referee_data_.shoot_frequency,
                               data_.manual_to_referee_data_.det_armor_target,
                               data_.manual_to_referee_data_.det_color == rm_msgs::StatusChangeRequest::RED);
}

void HeroReferee::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  RobotReferee::manualDataCallBack(data);
}

}  // namespace rm_referee
