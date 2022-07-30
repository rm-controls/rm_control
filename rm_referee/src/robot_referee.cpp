//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/robot_referee.h"

namespace rm_referee
{
RobotReferee::RobotReferee(ros::NodeHandle& nh, Base& base) : RefereeBase(nh, base)
{
  ros::NodeHandle ui_nh(nh, "ui");
  RobotReferee::chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                                     &RobotReferee::chassisCmdDataCallback, this);
  RobotReferee::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                   &RobotReferee::gimbalCmdDataCallback, this);
  RobotReferee::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &RobotReferee::manualDataCallBack, this);
  trigger_change_ui_ = new TriggerChangeUi(ui_nh, base);
  time_change_ui_ = new TimeChangeUi(ui_nh, base);
  flash_ui_ = new FlashUi(ui_nh, base);
  fixed_ui_ = new FixedUi(ui_nh, base);
  add_ui_flag_ = true;
}

void RobotReferee::addUi()
{
  RefereeBase::addUi();
  ROS_INFO("time ui");
  time_change_ui_->add();
  usleep(200000);
  ROS_INFO("trigger ui");
  trigger_change_ui_->add();
  usleep(200000);
  ROS_INFO("fixed ui");
  fixed_ui_->add();
  usleep(200000);
}

void RobotReferee::robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data_,
                                           const ros::Time& last_get_)
{
  RefereeBase::robotStatusDataCallBack(game_robot_status_data_, last_get_);
  fixed_ui_->update();
}

void RobotReferee::powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_)
{
  RefereeBase::powerHeatDataCallBack(power_heat_data_, last_get_);
}

void RobotReferee::robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_)
{
  RefereeBase::robotHurtDataCallBack(robot_hurt_data_, last_get_);
  flash_ui_->update("armor0", last_get_);
  flash_ui_->update("armor1", last_get_);
  flash_ui_->update("armor2", last_get_);
  flash_ui_->update("armor3", last_get_);
}

void RobotReferee::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  RefereeBase::chassisCmdDataCallback(data);

  flash_ui_->update("spin", ros::Time::now(), base_.chassis_cmd_data_.mode == rm_msgs::ChassisCmd::GYRO);

  if (base_.dbus_data_.s_l == rm_msgs::DbusData::MID && base_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    trigger_change_ui_->update("chassis", data->mode, false, 1, false);
  }
  else
  {
    trigger_change_ui_->update("chassis", data->mode,
                               base_.manual_to_referee_data_.power_limit_state == rm_common::PowerLimit::BURST, 0,
                               base_.manual_to_referee_data_.power_limit_state == rm_common::PowerLimit::CHARGE);
  }
}

void RobotReferee::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  RefereeBase::gimbalCmdDataCallback(data);
}

void RobotReferee::shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
{
  RefereeBase::shootCmdDataCallback(data);
}

void RobotReferee::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  RefereeBase::manualDataCallBack(data);
}

}  // namespace rm_referee
