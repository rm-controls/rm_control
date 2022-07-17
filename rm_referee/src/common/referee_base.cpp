//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/common/referee_base.h"

namespace rm_referee
{
RefereeBase::RefereeBase(ros::NodeHandle& nh, Data& data) : data_(data), nh_(nh)
{
  RefereeBase::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &RefereeBase::jointStateCallback, this);
  RefereeBase::actuator_state_sub_ =
      nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &RefereeBase::actuatorStateCallback, this);
  RefereeBase::dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &RefereeBase::dbusDataCallback, this);
  RefereeBase::chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                                    &RefereeBase::chassisCmdDataCallback, this);
  RefereeBase::vel2D_cmd_sub_ =
      nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RefereeBase::vel2DCmdDataCallback, this);
  RefereeBase::shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                                &RefereeBase::shootCmdDataCallback, this);
  RefereeBase::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &RefereeBase::gimbalCmdDataCallback, this);
  RefereeBase::cover_cmd_sub_ = nh.subscribe<std_msgs::Float64>("/controllers/cover_controller/command", 10,
                                                                &RefereeBase::coverCmdDataCallBack, this);
  RefereeBase::card_cmd_sub_ = nh.subscribe<rm_msgs::StateCmd>("/controllers/card_controller/command", 10,
                                                               &RefereeBase::cardCmdDataCallback, this);
  RefereeBase::engineer_cmd_sub_ =
      nh.subscribe<rm_msgs::EngineerCmd>("/engineer_cmd", 10, &RefereeBase::engineerCmdDataCallback, this);
  RefereeBase::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &RefereeBase::manualDataCallBack, this);
  if (data_.base_.robot_id_ == rm_common::RobotId::RED_RADAR || data_.base_.robot_id_ == rm_common::RobotId::BLUE_RADAR)
    RefereeBase::radar_date_sub_ =
        nh.subscribe<std_msgs::Int8MultiArray>("/data", 10, &RefereeBase::radarDataCallBack, this);
}

void RefereeBase::robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data_,
                                          const ros::Time& last_get_)
{
}
void RefereeBase::capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, const ros::Time& last_get_)
{
}
void RefereeBase::powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_)
{
}
void RefereeBase::robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_)
{
}
void RefereeBase::addUi()
{
}
void RefereeBase::run()
{
}
void RefereeBase::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  data_.joint_state_ = *joint_state;
}
void RefereeBase::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
  data_.actuator_state_ = *data;
}
void RefereeBase::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  data_.dbus_data_ = *data;
}
void RefereeBase::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  data_.chassis_cmd_data_ = *data;
}
void RefereeBase::vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
{
  data_.vel2d_cmd_data_ = *data;
}
void RefereeBase::shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
{
  data_.shoot_cmd_data_ = *data;
}
void RefereeBase::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  data_.gimbal_cmd_data_ = *data;
}
void RefereeBase::coverCmdDataCallBack(const std_msgs::Float64::ConstPtr& data)
{
  data_.cover_state_ = *data;
}
void RefereeBase::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
  data_.card_cmd_data_ = *data;
}
void RefereeBase::engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data)
{
  data_.engineer_cmd_data_ = *data;
}
void RefereeBase::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  data_.manual_to_referee_data_ = *data;
}
void RefereeBase::radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data)
{
  data_.radar_data_ = data->data[0] * 10 + data->data[1];
}

}  // namespace rm_referee
