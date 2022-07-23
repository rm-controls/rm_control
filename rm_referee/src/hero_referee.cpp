//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/hero_referee.h"

namespace rm_referee
{
HeroReferee::HeroReferee(ros::NodeHandle& nh, Data& data) : RobotReferee(nh, data)
{
  HeroReferee::chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                                    &HeroReferee::chassisCmdDataCallback, this);
  HeroReferee::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &HeroReferee::gimbalCmdDataCallback, this);
  HeroReferee::cover_cmd_sub_ = nh.subscribe<std_msgs::Float64>("/controllers/cover_controller/command", 10,
                                                                &HeroReferee::coverCmdDataCallBack, this);
  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
}
void HeroReferee::run()
{
  RobotReferee::run();
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

void HeroReferee::powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_)
{
  RobotReferee::powerHeatDataCallBack(power_heat_data_, last_get_);
}

void HeroReferee::robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_)
{
  RobotReferee::robotHurtDataCallBack(robot_hurt_data_, last_get_);
}

void HeroReferee::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  RobotReferee::chassisCmdDataCallback(data);
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
    trigger_change_ui_->update("target", switch_detection_srv_->getTarget(),
                               data_.manual_to_referee_data_.shoot_frequency == rm_common::HeatLimit::BURST,
                               switch_detection_srv_->getArmorTarget(),
                               switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
  else
    trigger_change_ui_->update("target", data_.manual_to_referee_data_.gimbal_eject,
                               data_.manual_to_referee_data_.shoot_frequency, switch_detection_srv_->getArmorTarget(),
                               switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
}

void HeroReferee::coverCmdDataCallBack(const std_msgs::Float64::ConstPtr& data)
{
  RobotReferee::coverCmdDataCallBack(data);
}
}  // namespace rm_referee
