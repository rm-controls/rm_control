//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/flash_ui.h"

namespace rm_referee
{
void ArmorFlashUi::display(const ros::Time& time)
{
  if (hurt_type_ == 0x00 && armor_id_ == getArmorId())
  {
    updateArmorPosition();
    graph_->display(time, true, true);
    graph_->sendUi(time);
    hurt_type_ = 9;
  }
  else
  {
    graph_->display(time, false, true);
    graph_->sendUi(time);
  }
}

void ArmorFlashUi::updateArmorPosition()
{
  geometry_msgs::TransformStamped yaw2base;
  double roll, pitch, yaw;
  try
  {
    yaw2base = tf_buffer_.lookupTransform("yaw", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
  }
  quatToRPY(yaw2base.transform.rotation, roll, pitch, yaw);
  if (getArmorId() == 0 || getArmorId() == 2)
  {
    graph_->setStartX(static_cast<int>((960 + 340 * sin(getArmorId() * M_PI_2 + yaw))));
    graph_->setStartY(static_cast<int>((540 + 340 * cos(getArmorId() * M_PI_2 + yaw))));
  }
  else
  {
    graph_->setStartX(static_cast<int>((960 + 340 * sin(-getArmorId() * M_PI_2 + yaw))));
    graph_->setStartY(static_cast<int>((540 + 340 * cos(-getArmorId() * M_PI_2 + yaw))));
  }
}

uint8_t ArmorFlashUi::getArmorId()
{
  if (graph_name_ == "armor0")
    return 0;
  else if (graph_name_ == "armor1")
    return 1;
  else if (graph_name_ == "armor2")
    return 2;
  else if (graph_name_ == "armor3")
    return 3;
  return 9;
}

void ArmorFlashUi::updateRobotHurtData(const rm_msgs::RobotHurt data, const ros::Time& last_get_data_time)
{
  hurt_type_ = data.hurt_type;
  armor_id_ = data.armor_id;
  display(last_get_data_time);
}

void CoverFlashUi::display(const ros::Time& time)
{
  if (!cover_state_)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  graph_->display(time, cover_state_, true);
  graph_->sendUi(time);
}

void CoverFlashUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data,
                                       const ros::Time& last_get_data_time)
{
  cover_state_ = data->cover_state;
  display(last_get_data_time);
}

void SpinFlashUi::display(const ros::Time& time)
{
  if (chassis_mode_ != rm_msgs::ChassisCmd::GYRO)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  graph_->display(time, chassis_mode_ == rm_msgs::ChassisCmd::GYRO);
  graph_->sendUi(time);
}

void SpinFlashUi::updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_data_time)
{
  chassis_mode_ = data->mode;
  display(last_get_data_time);
}
}  // namespace rm_referee
