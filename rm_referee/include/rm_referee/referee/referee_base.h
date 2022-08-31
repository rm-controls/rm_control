//
// Created by ljq on 2021/12/3.
//

#pragma once

#include "rm_referee/common/ui.h"

#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>

namespace rm_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(ros::NodeHandle& nh, Base& base);
  virtual void run();
  virtual void addUi();

  // unpack call back
  virtual void robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data_,
                                       const ros::Time& last_get_);
  virtual void gameStatusDataCallBack(const rm_msgs::GameStatus& game_status_data_, const ros::Time& last_get_);
  virtual void capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, const ros::Time& last_get_);
  virtual void powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_);
  virtual void robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_);
  virtual void interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data_, const ros::Time& last_get_);
  virtual void eventDataCallBack(const rm_msgs::EventData& event_data_, const ros::Time& last_get_);

  // sub call back
  virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data);
  virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data);
  virtual void chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data);
  virtual void vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data);
  virtual void shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data);
  virtual void gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data);
  virtual void cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data);
  virtual void engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data);
  virtual void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data);
  virtual void radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data);

  ros::Subscriber joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber chassis_cmd_sub_;
  ros::Subscriber vel2D_cmd_sub_;
  ros::Subscriber shoot_cmd_sub_;
  ros::Subscriber gimbal_cmd_sub_;
  ros::Subscriber detection_status_sub_;
  ros::Subscriber card_cmd_sub_;
  ros::Subscriber calibration_status_sub_;
  ros::Subscriber engineer_cmd_sub_;
  ros::Subscriber radar_date_sub_;
  ros::Subscriber manual_data_sub_;

  Base& base_;
  bool add_ui_flag_ = false;
  Graph* interactive_data_sender_;
  ros::NodeHandle nh_;
};
}  // namespace rm_referee
