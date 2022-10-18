//
// Created by peter on 2021/7/20.
//

#include "rm_referee/common/ui.h"
#include "rm_common/decision/power_limit.h"

namespace rm_referee
{
int UiBase::id_(2);
UiBase::UiBase(ros::NodeHandle& nh, Base& base, const std::string& ui_type) : base_(base), tf_listener_(tf_buffer_)
{
  XmlRpc::XmlRpcValue rpc_value;
  if (!nh.getParam(ui_type, rpc_value))
  {
    ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
    return;
  }
  try
  {
    for (int i = 0; i < static_cast<int>(rpc_value.size()); i++)
    {
      if (rpc_value[i]["name"] == "chassis")
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, 1)));
      else
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("Wrong ui parameter: %s", e.getMessage().c_str());
  }
  for (auto graph : graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::DELETE);
}

void UiBase::add()
{
  graph_->setOperation(rm_referee::GraphOperation::ADD);
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

TriggerChangeUi::TriggerChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name)
  : UiBase(nh, base, "trigger_change")
{
  for (auto graph : graph_vector_)
    if (graph.first == graph_name)
      graph_ = graph.second;
}

void TriggerChangeUi::setContent(const std::string& content)
{
  graph_->setContent(content);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display();
  graph_->sendUi(ros::Time::now());
}

void TriggerChangeUi::display()
{
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display();
  graph_->sendUi(ros::Time::now());
}

ChassisTriggerChangeUi::ChassisTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "chassis")
{
  if (base.robot_id_ == rm_referee::RobotId::RED_ENGINEER || base.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
    graph_->setContent("raw");
  else
    graph_->setContent("follow");
}

void ChassisTriggerChangeUi::display()
{
  if (s_l_ == rm_msgs::DbusData::MID && s_r_ == rm_msgs::DbusData::UP)
    updateConfig(chassis_mode_, false, 1, false);
  else
    updateConfig(chassis_mode_, power_limit_state_ == rm_common::PowerLimit::BURST, 0,
                 power_limit_state_ == rm_common::PowerLimit::CHARGE);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::displayInCapacity()
{
  if (key_ctrl_ && key_shift_ && key_b_)
    updateConfig(254, 0);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  if (main_mode == 254)
  {
    graph_->setContent("Cap reset");
    graph_->setColor(rm_referee::GraphColor::YELLOW);
    return;
  }
  {
    graph_->setContent(getChassisState(main_mode));
  }
  if (sub_mode == 1)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else if (sub_mode == 0)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string ChassisTriggerChangeUi::getChassisState(uint8_t mode)
{
  if (mode == rm_msgs::ChassisCmd::RAW)
    return "raw";
  else if (mode == rm_msgs::ChassisCmd::FOLLOW)
    return "follow";
  else if (mode == rm_msgs::ChassisCmd::GYRO)
    return "gyro";
  else if (mode == rm_msgs::ChassisCmd::TWIST)
    return "twist";
  else
    return "error";
}

void ChassisTriggerChangeUi::updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data)
{
  chassis_mode_ = data->mode;
  display();
}

void ChassisTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  power_limit_state_ = data->power_limit_state;
}

void ChassisTriggerChangeUi::updateDbusData(const rm_msgs::DbusData::ConstPtr data)
{
  s_l_ = data->s_l;
  s_r_ = data->s_r;
  key_ctrl_ = data->key_ctrl;
  key_shift_ = data->key_shift;
  key_b_ = data->key_b;
}

void ChassisTriggerChangeUi::updateCapacityData(const rm_msgs::CapacityData data)
{
  displayInCapacity();
}

ShooterTriggerChangeUi::ShooterTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "shooter")
{
  graph_->setContent("0");
}

void ShooterTriggerChangeUi::display()
{
  updateConfig(shooter_mode_, 0, shoot_frequency_, false);
  TriggerChangeUi::display();
}

void ShooterTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getShooterState(main_mode));
  if (sub_mode == rm_common::HeatLimit::LOW)
    graph_->setColor(rm_referee::GraphColor::WHITE);
  else if (sub_mode == rm_common::HeatLimit::HIGH)
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  else if (sub_mode == rm_common::HeatLimit::BURST)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
}

std::string ShooterTriggerChangeUi::getShooterState(uint8_t mode)
{
  if (mode == rm_msgs::ShootCmd::READY)
    return "ready";
  else if (mode == rm_msgs::ShootCmd::PUSH)
    return "push";
  else if (mode == rm_msgs::ShootCmd::STOP)
    return "stop";
  else
    return "error";
}

void ShooterTriggerChangeUi::updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data)
{
  shooter_mode_ = data->mode;
  display();
}

void ShooterTriggerChangeUi::updateManualCmdData(rm_msgs::ManualToReferee::ConstPtr data)
{
  shoot_frequency_ = data->shoot_frequency;
}

GimbalTriggerChangeUi::GimbalTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "gimbal")
{
  graph_->setContent("0");
}

void GimbalTriggerChangeUi::display()
{
  updateConfig(gimbal_mode_, gimbal_eject_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void GimbalTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getGimbalState(main_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string GimbalTriggerChangeUi::getGimbalState(uint8_t mode)
{
  if (mode == rm_msgs::GimbalCmd::DIRECT)
    return "direct";
  else if (mode == rm_msgs::GimbalCmd::RATE)
    return "rate";
  else if (mode == rm_msgs::GimbalCmd::TRACK)
    return "track";
  else
    return "error";
}

void GimbalTriggerChangeUi::updateGimbalCmdData(const rm_msgs::GimbalCmd::ConstPtr data)
{
  gimbal_mode_ = data->mode;
  display();
}

void GimbalTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  gimbal_eject_ = data->gimbal_eject;
}

TargetTriggerChangeUi::TargetTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "target")
{
  for (auto graph : graph_vector_)
    graph_->setContent("armor");
  if (base_.robot_color_ == "red")
    graph_->setColor(rm_referee::GraphColor::CYAN);
  else
    graph_->setColor(rm_referee::GraphColor::PINK);
}

void TargetTriggerChangeUi::display()
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
    updateConfig(det_target_, shoot_frequency_ == rm_common::HeatLimit::BURST, det_armor_target_,
                 det_color_ == rm_msgs::StatusChangeRequest::RED);
  else
    updateConfig(gimbal_eject_, shoot_frequency_, det_armor_target_, det_color_ == rm_msgs::StatusChangeRequest::RED);
  TriggerChangeUi::display();
}

void TargetTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getTargetState(main_mode, sub_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::CYAN);
}

std::string TargetTriggerChangeUi::getTargetState(uint8_t target, uint8_t armor_target)
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
  {
    if (target == rm_msgs::StatusChangeRequest::BUFF)
      return "buff";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR && armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "armor_all";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR &&
             armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "armor_base";
    else
      return "error";
  }
  else
  {
    if (target == 1)
      return "eject";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "all";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "base";
    else
      return "error";
  }
}

void TargetTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  det_target_ = data->det_target;
  shoot_frequency_ = data->shoot_frequency;
  det_armor_target_ = data->det_armor_target;
  det_color_ = data->det_color;
  gimbal_eject_ = data->gimbal_eject;
}

void TargetTriggerChangeUi::updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data)
{
  display();
}

void FixedUi::display()
{
  for (auto graph : graph_vector_)
  {
    graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
    graph.second->display();
    graph.second->sendUi(ros::Time::now());
  }
}

TimeChangeUi::TimeChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name)
  : UiBase(nh, base, "time_change")
{
  for (auto graph : graph_vector_)
    if (graph.first == graph_name)
      graph_ = graph.second;
}

void TimeChangeUi::display(const ros::Time& time)
{
  graph_->display(time);
  graph_->sendUi(ros::Time::now());
}

CapacitorTimeChangeUI::CapacitorTimeChangeUI(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "capacitor")
{
}

void CapacitorTimeChangeUI::add()
{
  if (cap_power_ != 0.)
  {
    graph_->setOperation(rm_referee::GraphOperation::ADD);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }
}

void CapacitorTimeChangeUI::display(const ros::Time& time)
{
  updateData();
  TimeChangeUi::display(time);
}

void CapacitorTimeChangeUI::updateData()
{
  if (cap_power_ != 0.)
  {
    if (cap_power_ > 0.)
    {
      graph_->setStartX(610);
      graph_->setStartY(100);

      graph_->setEndX(610 + 600 * cap_power_);
      graph_->setEndY(100);
    }
    else
      return;
    if (cap_power_ < 0.3)
      graph_->setColor(rm_referee::GraphColor::ORANGE);
    else if (cap_power_ > 0.7)
      graph_->setColor(rm_referee::GraphColor::GREEN);
    else
      graph_->setColor(rm_referee::GraphColor::YELLOW);
    graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  }
}

void CapacitorTimeChangeUI::updateCapacityData(const rm_msgs::CapacityData data, const ros::Time& time)
{
  cap_power_ = data.cap_power;
  display(time);
}

EffortTimeChangeUI::EffortTimeChangeUI(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "effort")
{
}

void EffortTimeChangeUI::display(const ros::Time& time)
{
}

void EffortTimeChangeUI::updateData()
{
  char data_str[30] = { ' ' };
  sprintf(data_str, "%s:%.2f N.m", joint_name_.c_str(), joint_effort_);
  graph_->setContent(data_str);
  if (joint_effort_ > 20.)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (joint_effort_ < 10.)
    graph_->setColor(rm_referee::GraphColor::GREEN);
  else
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void EffortTimeChangeUI::updateJointStateData(const sensor_msgs::JointState::ConstPtr data){

};

ProgressTimeChangeUI::ProgressTimeChangeUI(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "progress")
{
}

void ProgressTimeChangeUI::updateData()
{
  char data_str[30] = { ' ' };
  if (total_steps_ != 0)
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  else
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  graph_->setContent(data_str);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void ProgressTimeChangeUI::progressDataCallBack(uint32_t finished_data, uint32_t total_steps, std::string step_name)
{
  finished_data_ = finished_data;
  total_steps_ = total_steps;
  if (step_name_ != step_name)
  {
    step_name_ = step_name;
    update();
  }
}

DartStatusTimeChangeUI::DartStatusTimeChangeUI(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "dart")
{
}

void DartStatusTimeChangeUI::updateData()
{
  char data_str[30] = { ' ' };
  if (dart_launch_opening_status_ == 1)
  {
    sprintf(data_str, "Dart Status: Close");
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  }
  else if (dart_launch_opening_status_ == 2)
  {
    sprintf(data_str, "Dart Status: Changing");
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  }
  else if (dart_launch_opening_status_ == 0)
  {
    sprintf(data_str, "Dart Open!");
    graph_->setColor(rm_referee::GraphColor::GREEN);
  }
  graph_->setContent(data_str);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void DartStatusTimeChangeUI::dartLaunchOpeningStatusCallBack(uint8_t dart_launch_opening_status)
{
  dart_launch_opening_status_ = dart_launch_opening_status;
}

OreRemindTimeChangeUI::OreRemindTimeChangeUI(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "ore_remind")
{
}

void OreRemindTimeChangeUI::updateData()
{
  char data_str[30] = { ' ' };
  int time = stage_remain_time_;
  if (time < 420 && time > 417)
    sprintf(data_str, "Ore will released after 15s");
  else if (time < 272 && time > 269)
    sprintf(data_str, "Ore will released after 30s");
  else if (time < 252 && time > 249)
    sprintf(data_str, "Ore will released after 10s");
  else
    return;
  graph_->setContent(data_str);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void OreRemindTimeChangeUI::stageRemainTimeCallBack(uint16_t stage_remain_time)
{
  stage_remain_time_ = stage_remain_time;
}

FlashUi::FlashUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name) : UiBase(nh, base, "flash")
{
  for (auto graph : graph_vector_)
    if (graph.first == graph_name)
      graph_ = graph.second;
}

void FlashUi::display(const ros::Time& time, bool state)
{
  if (state)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  else
    graph_->display(time, !state);
  graph_->sendUi(time);
}

void FlashUi::updateData()
{
}

CoverFlashUI::CoverFlashUI(ros::NodeHandle& nh, Base& base) : FlashUi(nh, base, "cover")
{
}

void CoverFlashUI::display(const ros::Time& time)
{
  if (cover_state_)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  graph_->display(time, !cover_state_, true);
  graph_->sendUi(time);
}

void CoverFlashUI::coverStateCallBack(uint8_t cover_state)
{
  cover_state = cover_state_;
  display(ros::Time::now());
}

}  // namespace rm_referee
