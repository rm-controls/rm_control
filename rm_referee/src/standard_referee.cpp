//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/standard_referee.h"

namespace rm_referee
{
StandardReferee::StandardReferee(ros::NodeHandle& nh, Data& data) : HeroReferee(nh, data)
{
  StandardReferee::chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                                        &StandardReferee::chassisCmdDataCallback, this);
  StandardReferee::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                      &StandardReferee::gimbalCmdDataCallback, this);
  StandardReferee::cover_cmd_sub_ = nh.subscribe<std_msgs::Float64>("/controllers/cover_controller/command", 10,
                                                                    &StandardReferee::coverCmdDataCallBack, this);
}
void StandardReferee::run()
{
  RobotReferee::run();
}

void StandardReferee::capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, const ros::Time& last_get_)
{
  HeroReferee::capacityDataCallBack(capacity_data_, last_get_);
}

void StandardReferee::powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_)
{
  HeroReferee::powerHeatDataCallBack(power_heat_data_, last_get_);
}

void StandardReferee::robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_)
{
  HeroReferee::robotHurtDataCallBack(robot_hurt_data_, last_get_);
}

void StandardReferee::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  HeroReferee::chassisCmdDataCallback(data);
}

void StandardReferee::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  HeroReferee::gimbalCmdDataCallback(data);
}

void StandardReferee::shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
{
  HeroReferee::shootCmdDataCallback(data);
}

void StandardReferee::coverCmdDataCallBack(const std_msgs::Float64::ConstPtr& data)
{
  HeroReferee::coverCmdDataCallBack(data);
  flash_ui_->update("cover", ros::Time::now(), !data_.manual_to_referee_data_.cover_state);
}

}  // namespace rm_referee
