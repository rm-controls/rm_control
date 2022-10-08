//
// Created by peter on 2021/7/20.
//

#include "rm_referee/common/ui.h"
#include "rm_common/decision/power_limit.h"

namespace rm_referee
{
int UiBase::id_(2);
UiBase::UiBase(ros::NodeHandle& nh, DataTranslation& data_translation, const std::string& ui_type)
  : data_translation_(data_translation), tf_listener_(tf_buffer_)
{
  XmlRpc::XmlRpcValue rpc_value;
  if (!nh.getParam(ui_type, rpc_value))
  {
    ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
    return;
  }
  try
  {
    for (int i = 0; i < static_cast<int>(rpc_value["special"].size()); i++)
    {
      if (rpc_value[i]["name"] == "chassis")
        special_graph_vector_.insert(std::pair<std::string, Graph*>(
            rpc_value["special"][i]["name"], new Graph(rpc_value[i]["special"]["config"], data_translation_, 1)));
      else
        special_graph_vector_.insert(std::pair<std::string, Graph*>(
            rpc_value[i]["special"]["name"], new Graph(rpc_value[i]["special"]["config"], data_translation_, id_++)));
    }
    for (int i = 0; i < static_cast<int>(rpc_value["normal"].size()); i++)
    {
      normal_graph_vector_.insert(std::pair<std::string, Graph*>(
          rpc_value[i]["normal"]["name"], new Graph(rpc_value[i]["normal"]["config"], data_translation_, id_++)));
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("Wrong ui parameter: %s", e.getMessage().c_str());
  }
  for (auto graph : special_graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::DELETE);
  for (auto graph : normal_graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::DELETE);
}

void UiBase::add()
{
  for (auto graph : normal_graph_vector_)
  {
    graph.second->setOperation(rm_referee::GraphOperation::ADD);
    graph.second->display(true);
    graph.second->sendUi(ros::Time::now());
  }
}

TriggerChangeUi::TriggerChangeUi(ros::NodeHandle& nh, DataTranslation& data_translation)
  : UiBase(nh, data_translation, "trigger_change")
{
  for (auto graph : normal_graph_vector_)
    graph.second->setContent("0");
}

void TriggerChangeUi::update(const std::string& graph_name, const std::string& content)
{
  auto graph = normal_graph_vector_.find(graph_name);
  if (graph != normal_graph_vector_.end())
  {
    if (graph_name == "stone")
    {
      if (content == "0")
        graph->second->setContent("upper");
      else
        graph->second->setContent("lower");
    }
    else
      graph->second->setContent(content);
    graph->second->setOperation(rm_referee::GraphOperation::UPDATE);
    graph->second->display();
    graph->second->sendUi(ros::Time::now());
  }
}

ChassisTriggerChangeUi::ChassisTriggerChangeUi(ros::NodeHandle& nh, DataTranslation& data_translation)
  : TriggerChangeUi(nh, data_translation)
{
  for (auto graph : special_graph_vector_)
    if (graph.first == "chassis")
      chassis_graph_ = graph.second;
  if (data_translation.robot_id_ == rm_referee::RobotId::RED_ENGINEER ||
      data_translation.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
    chassis_graph_->setContent("raw");
  else
    chassis_graph_->setContent("follow");
}
void ChassisTriggerChangeUi::update()
{
  if (s_l_ == rm_msgs::DbusData::MID && s_r_ == rm_msgs::DbusData::UP)
  {
    update("chassis", chassis_mode_, false, 1, false);
  }
  else
  {
    update("chassis", chassis_mode_, power_limit_state_ == rm_common::PowerLimit::BURST, 0,
           power_limit_state_ == rm_common::PowerLimit::CHARGE);
  }
  if (key_ctrl_ && key_shift_ && key_b_)
  {
    update("chassis", 254, 0);
  }
}

void ChassisTriggerChangeUi::update(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode,
                                    bool sub_flag)
{
  if (main_mode == 254)
  {
    chassis_graph_->setContent("Cap reset");
    chassis_graph_->setColor(rm_referee::GraphColor::YELLOW);
    return;
  }
  chassis_graph_->setContent(getChassisState(main_mode));
  if (main_flag)
    chassis_graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    chassis_graph_->setColor(rm_referee::GraphColor::GREEN);
  else if (sub_mode == 1)
    chassis_graph_->setColor(rm_referee::GraphColor::PINK);
  else
    chassis_graph_->setColor(rm_referee::GraphColor::WHITE);

  chassis_graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  chassis_graph_->displayTwice(true);
  chassis_graph_->sendUi(ros::Time::now());
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

void ChassisTriggerChangeUi::updateChassisMode(uint8_t mode)
{
  chassis_mode_ = mode;
}
//
// ShooterTriggerChangeUi::ShooterTriggerChangeUi(ros::NodeHandle& nh) : TriggerChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "shooter")
//      shooter_graph_ = graph.second;
//  shooter_graph_->setContent("0");
//}
//
// void ShooterTriggerChangeUi::updateShooter(const std::string& graph_name, uint8_t main_mode, bool main_flag,
//                                           uint8_t sub_mode, bool sub_flag)
//{
//  shooter_graph_->setContent(getShooterState(main_mode));
//  if (sub_mode == rm_common::HeatLimit::LOW)
//    shooter_graph_->setColor(rm_referee::GraphColor::WHITE);
//  else if (sub_mode == rm_common::HeatLimit::HIGH)
//    shooter_graph_->setColor(rm_referee::GraphColor::YELLOW);
//  else if (sub_mode == rm_common::HeatLimit::BURST)
//    shooter_graph_->setColor(rm_referee::GraphColor::ORANGE);
//
//  shooter_graph_->setOperation(rm_referee::GraphOperation::UPDATE);
//  shooter_graph_->display();
//  shooter_graph_->sendUi(ros::Time::now());
//}
//
// std::string ShooterTriggerChangeUi::getShooterState(uint8_t mode)
//{
//  if (mode == rm_msgs::ShootCmd::READY)
//    return "ready";
//  else if (mode == rm_msgs::ShootCmd::PUSH)
//    return "push";
//  else if (mode == rm_msgs::ShootCmd::STOP)
//    return "stop";
//  else
//    return "error";
//}
//
// void ShooterTriggerChangeUi::updateShooterMode(uint8_t mode)
//{
//  shooter_mode_ = mode;
//}
//
// GimbalTriggerChangeUi::GimbalTriggerChangeUi(ros::NodeHandle& nh) : TriggerChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "gimbal")
//      gimbal_graph_ = graph.second;
//  gimbal_graph_->setContent("0");
//}
//
// void GimbalTriggerChangeUi::updateGimbal(const std::string& graph_name, uint8_t main_mode, bool main_flag,
//                                         uint8_t sub_mode, bool sub_flag)
//{
//  gimbal_graph_->setContent(getGimbalState(main_mode));
//  if (main_flag)
//    gimbal_graph_->setColor(rm_referee::GraphColor::ORANGE);
//  else
//    gimbal_graph_->setColor(rm_referee::GraphColor::WHITE);
//}
//
// std::string GimbalTriggerChangeUi::getGimbalState(uint8_t mode)
//{
//  if (mode == rm_msgs::GimbalCmd::DIRECT)
//    return "direct";
//  else if (mode == rm_msgs::GimbalCmd::RATE)
//    return "rate";
//  else if (mode == rm_msgs::GimbalCmd::TRACK)
//    return "track";
//  else
//    return "error";
//}
//
// void GimbalTriggerChangeUi::updateGimbalMode(uint8_t mode)
//{
//  gimbal_mode_ = mode;
//}
//
// TargetTriggerChangeUi::TargetTriggerChangeUi(ros::NodeHandle& nh) : TriggerChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "target")
//      target_graph_ = graph.second;
//  target_graph_->setContent("armor");
//  if (robot_color_ == "red")
//    target_graph_->setColor(rm_referee::GraphColor::CYAN);
//  else
//    target_graph_->setColor(rm_referee::GraphColor::PINK);
//}
//
// void TargetTriggerChangeUi::updateTarget(const std::string& graph_name, uint8_t main_mode, bool main_flag,
//                                         uint8_t sub_mode, bool sub_flag)
//{
//  target_graph_->setContent(getTargetState(main_mode, sub_mode));
//  if (main_flag)
//    target_graph_->setColor(rm_referee::GraphColor::ORANGE);
//  else if (sub_flag)
//    target_graph_->setColor(rm_referee::GraphColor::PINK);
//  else
//    target_graph_->setColor(rm_referee::GraphColor::CYAN);
//}
//
// std::string TargetTriggerChangeUi::getTargetState(uint8_t target, uint8_t armor_target)
//{
//  if (robot_id_ != rm_referee::RobotId::BLUE_HERO && robot_id_ != rm_referee::RobotId::RED_HERO)
//  {
//    if (target == rm_msgs::StatusChangeRequest::BUFF)
//      return "buff";
//    else if (target == rm_msgs::StatusChangeRequest::ARMOR && armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
//      return "armor_all";
//    else if (target == rm_msgs::StatusChangeRequest::ARMOR &&
//             armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
//      return "armor_base";
//    else
//      return "error";
//  }
//  else
//  {
//    if (target == 1)
//      return "eject";
//    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
//      return "all";
//    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
//      return "base";
//    else
//      return "error";
//  }
//}
// void FixedUi::update()
//{
//  for (auto graph : graph_vector_)
//  {
//    graph.second->updatePosition(getShootSpeedIndex());
//    graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
//    graph.second->display();
//    graph.second->sendUi(ros::Time::now());
//  }
//}
//
// int FixedUi::getShootSpeedIndex()
//{
//  if (robot_id_ != rm_referee::RobotId::BLUE_HERO && robot_id_ != rm_referee::RobotId::RED_HERO)
//  {
//    if (speed_limit_ == 15)
//      return 0;
//    else if (speed_limit_ == 18)
//      return 1;
//    else if (speed_limit_ == 30)
//      return 2;
//  }
//  return 0;
//}
//
// void FixedUi::updateSpeedLimit(int speed_limit)
//{
//  speed_limit_ = speed_limit;
//}
//
// ArmorFlashUI::ArmorFlashUI(ros::NodeHandle& nh, std::string armor_name) : FlashUi(nh)
//{
//  armor_name_ = armor_name;
//  for (auto graph : graph_vector_)
//    if (graph.first == armor_name)
//      armor_graph_ = graph.second;
//}
//
// void ArmorFlashUI::update(const std::string& name, const ros::Time& time, bool state)
//{
//  if (base_.robot_hurt_data_.hurt_type == 0x00 && base_.robot_hurt_data_.armor_id == getArmorId(graph->first))
//  {
//    updateArmorPosition(armor_graph_);
//    armor_graph_->display(time, true, true);
//    armor_graph_->sendUi(time);
//    base_.robot_hurt_data_.hurt_type = 9;
//  }
//  else
//  {
//    armor_graph_->display(time, false, true);
//    armor_graph_->sendUi(time);
//  }
//}
//
// void ArmorFlashUI::updateArmorPosition(Graph* graph)
//{
//  geometry_msgs::TransformStamped yaw_2_baselink;
//  double roll, pitch, yaw;
//  try
//  {
//    yaw_2_baselink = tf_buffer_.lookupTransform("yaw", "base_link", ros::Time(0));
//  }
//  catch (tf2::TransformException& ex)
//  {
//  }
//  quatToRPY(yaw_2_baselink.transform.rotation, roll, pitch, yaw);
//  if (getArmorId(armor_name_) == 0 || getArmorId(armor_name_) == 2)
//  {
//    graph->setStartX(static_cast<int>((960 + 340 * sin(getArmorId(armor_name_) * M_PI_2 + yaw))));
//    graph->setStartY(static_cast<int>((540 + 340 * cos(getArmorId(armor_name_) * M_PI_2 + yaw))));
//  }
//  else
//  {
//    graph->setStartX(static_cast<int>((960 + 340 * sin(-getArmorId(armor_name_) * M_PI_2 + yaw))));
//    graph->setStartY(static_cast<int>((540 + 340 * cos(-getArmorId(armor_name_) * M_PI_2 + yaw))));
//  }
//}
//
// uint8_t ArmorFlashUI::getArmorId(const std::string& name)
//{
//  if (name == "armor0")
//    return 0;
//  else if (name == "armor1")
//    return 1;
//  else if (name == "armor2")
//    return 2;
//  else if (name == "armor3")
//    return 3;
//  return 9;
//}
//
// void FlashUi::update(const std::string& name, const ros::Time& time, bool state)
//{
//  auto graph = graph_vector_.find(name);
//  if (graph == graph_vector_.end())
//    return;
//  if (name.find("armor") != std::string::npos)
//  {
//    if (base_.robot_hurt_data_.hurt_type == 0x00 && base_.robot_hurt_data_.armor_id == getArmorId(graph->first))
//    {
//      updateArmorPosition(graph->first, graph->second);
//      graph->second->display(time, true, true);
//      graph->second->sendUi(time);
//      base_.robot_hurt_data_.hurt_type = 9;
//    }
//    else
//    {
//      graph->second->display(time, false, true);
//      graph->second->sendUi(time);
//    }
//  }
//  else
//  {
//    if (name == "aux")
//      updateChassisGimbalDate(base_.joint_state_.position[8], graph->second);
//    if (state)
//      graph->second->setOperation(rm_referee::GraphOperation::DELETE);
//
//    if (name == "cover")
//      graph->second->display(time, !state, true);
//    else
//      graph->second->display(time, !state);
//    graph->second->sendUi(time);
//  }
//}
//
// void FlashUi::updateChassisGimbalDate(const double yaw_joint_, Graph* graph)
//{
//  double cover_yaw_joint = yaw_joint_;
//  while (abs(cover_yaw_joint) > 2 * M_PI)
//  {
//    cover_yaw_joint += cover_yaw_joint > 0 ? -2 * M_PI : 2 * M_PI;
//  }
//  graph->setStartX(960 - 50 * sin(cover_yaw_joint));
//  graph->setStartY(540 + 50 * cos(cover_yaw_joint));
//
//  graph->setEndX(960 - 100 * sin(cover_yaw_joint));
//  graph->setEndY(540 + 100 * cos(cover_yaw_joint));
//}
//
// void FlashUi::updateArmorPosition(const std::string& name, Graph* graph)
//{
//  geometry_msgs::TransformStamped yaw_2_baselink;
//  double roll, pitch, yaw;
//  try
//  {
//    yaw_2_baselink = tf_buffer_.lookupTransform("yaw", "base_link", ros::Time(0));
//  }
//  catch (tf2::TransformException& ex)
//  {
//  }
//  quatToRPY(yaw_2_baselink.transform.rotation, roll, pitch, yaw);
//  if (getArmorId(name) == 0 || getArmorId(name) == 2)
//  {
//    graph->setStartX(static_cast<int>((960 + 340 * sin(getArmorId(name) * M_PI_2 + yaw))));
//    graph->setStartY(static_cast<int>((540 + 340 * cos(getArmorId(name) * M_PI_2 + yaw))));
//  }
//  else
//  {
//    graph->setStartX(static_cast<int>((960 + 340 * sin(-getArmorId(name) * M_PI_2 + yaw))));
//    graph->setStartY(static_cast<int>((540 + 340 * cos(-getArmorId(name) * M_PI_2 + yaw))));
//  }
//}
//
// uint8_t FlashUi::getArmorId(const std::string& name)
//{
//  if (name == "armor0")
//    return 0;
//  else if (name == "armor1")
//    return 1;
//  else if (name == "armor2")
//    return 2;
//  else if (name == "armor3")
//    return 3;
//  return 9;
//}
//
// void TimeChangeUi::add()
//{
//  for (auto graph : graph_vector_)
//  {
//    if (graph.first == "capacitor" && base_.capacity_data_.cap_power == 0.)
//      continue;
//    graph.second->setOperation(rm_referee::GraphOperation::ADD);
//    graph.second->display(true);
//    graph.second->sendUi(ros::Time::now());
//  }
//}
//
// void TimeChangeUi::update(const std::string& name, const ros::Time& time, double data)
//{
//  auto graph = graph_vector_.find(name);
//  if (graph != graph_vector_.end())
//  {
//    if (name == "capacitor")
//      setCapacitorData(*graph->second);
//    if (name == "effort")
//      setEffortData(*graph->second);
//    if (name == "progress")
//      setProgressData(*graph->second, data);
//    if (name == "temperature")
//      setTemperatureData(*graph->second);
//    if (name == "dart_status")
//      setDartStatusData(*graph->second);
//    graph->second->display(time);
//    graph->second->sendUi(ros::Time::now());
//  }
//}
//
// CapacitorTimeChangeUI::CapacitorTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "capacitor")
//      capacitor_graph_ = graph.second;
//}
//
// void CapacitorTimeChangeUI::add()
//{
//  if (cap_power_ != 0.)
//    capacitor_graph_->setOperation(rm_referee::GraphOperation::ADD);
//  capacitor_graph_->display(true);
//  capacitor_graph_->sendUi(ros::Time::now());
//}
//
// void CapacitorTimeChangeUI::updateCapPower(double cap_power)
//{
//  cap_power_ = cap_power;
//}
//
// void CapacitorTimeChangeUI::setCapacitorData()
//{
//  if (cap_power_ != 0.)
//  {
//    if (cap_power_ > 0.)
//    {
//      capacitor_graph_->setStartX(610);
//      capacitor_graph_->setStartY(100);
//
//      capacitor_graph_->setEndX(610 + 600 * cap_power_);
//      capacitor_graph_->setEndY(100);
//    }
//    else
//      return;
//    if (cap_power_ < 0.3)
//      capacitor_graph_->setColor(rm_referee::GraphColor::ORANGE);
//    else if (cap_power_ > 0.7)
//      capacitor_graph_->setColor(rm_referee::GraphColor::GREEN);
//    else
//      capacitor_graph_->setColor(rm_referee::GraphColor::YELLOW);
//    capacitor_graph_->setOperation(rm_referee::GraphOperation::UPDATE);
//  }
//}
//
// EffortTimeChangeUI::EffortTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "effort")
//      effort_graph_ = graph.second;
//}
//
// void EffortTimeChangeUI::updateEffort(std::string joint_name[], double joint_effort[])
//{
//  joint_name_ = joint_name;
//  joint_effort_ = joint_effort_;
//}
//
// void EffortTimeChangeUI::setEffortData()
//{
//  char data_str[30] = { ' ' };
//  int max_index = 0;
//  if (!joints_->joint_name_.empty())
//  {
//    for (int i = 0; i < static_cast<int>(joints_[i].joint_state_.effort.size()); ++i)
//      if ((test[i].joint_name_ == "joint1" || base_.joint_state_.name[i] == "joint2" ||
//           base_.joint_state_.name[i] == "joint3" || base_.joint_state_.name[i] == "joint4" ||
//           base_.joint_state_.name[i] == "joint5") &&
//          base_.joint_state_.effort[i] > base_.joint_state_.effort[max_index])
//        max_index = i;
//    if (max_index != 0)
//    {
//      sprintf(data_str, "%s:%.2f N.m", base_.joint_state_.name[max_index].c_str(), base_.joint_state_.effort[max_index]);
//      graph.setContent(data_str);
//      if (base_.joint_state_.effort[max_index] > 20.)
//        graph.setColor(rm_referee::GraphColor::ORANGE);
//      else if (base_.joint_state_.effort[max_index] < 10.)
//        graph.setColor(rm_referee::GraphColor::GREEN);
//      else
//        graph.setColor(rm_referee::GraphColor::YELLOW);
//      graph.setOperation(rm_referee::GraphOperation::UPDATE);
//    }
//  }
//}
//
// ProgressTimeChangeUI::ProgressTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "effort")
//      progress_graph_ = graph.second;
//}
//
// void ProgressTimeChangeUI::setProgressData(double data)
//{
//  char data_str[30] = { ' ' };
//  sprintf(data_str, " %.1f%%", data * 100.);
//  progress_graph_->setContent(data_str);
//  progress_graph_->setOperation(rm_referee::GraphOperation::UPDATE);
//}
//
// TemperatureTimeChangeUI::TemperatureTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "effort")
//      temperature_graph_ = graph.second;
//}
//
// void TemperatureTimeChangeUI::setTemperatureData()
//{
//  char data_str[30] = { ' ' };
//  for (int i = 0; i < static_cast<int>(base_.actuator_state_.name.size()); ++i)
//  {
//    if (base_.actuator_state_.name[i] == "right_finger_joint_motor")
//    {
//      sprintf(data_str, " %.1hhu C", base_.actuator_state_.temperature[i]);
//      graph.setContent(data_str);
//      if (base_.actuator_state_.temperature[i] > 70.)
//        graph.setColor(rm_referee::GraphColor::ORANGE);
//      else if (base_.actuator_state_.temperature[i] < 30.)
//        graph.setColor(rm_referee::GraphColor::GREEN);
//      else
//        graph.setColor(rm_referee::GraphColor::YELLOW);
//    }
//  }
//  graph.setOperation(rm_referee::GraphOperation::UPDATE);
//}
//
// OreRemindTimeChangeUI::OreRemindTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "ore")
//      ore_remind_graph_ = graph.second;
//}
//
// void OreRemindTimeChangeUI::setOreRemindData()
//{
//  char data_str[30] = { ' ' };
//  int time = base_.game_status_data_.stage_remain_time;
//  if (time < 420 && time > 417)
//    sprintf(data_str, "Ore will released after 15s");
//  else if (time < 272 && time > 269)
//    sprintf(data_str, "Ore will released after 30s");
//  else if (time < 252 && time > 249)
//    sprintf(data_str, "Ore will released after 10s");
//  else
//    return;
//  graph.setContent(data_str);
//  graph.setOperation(rm_referee::GraphOperation::UPDATE);
//}
//
// DartStatusTimeChangeUI::DartStatusTimeChangeUI(ros::NodeHandle& nh) : TimeChangeUi(nh)
//{
//  for (auto graph : graph_vector_)
//    if (graph.first == "dart")
//      dart_status_graph_ = graph.second;
//}
//
// void DartStatusTimeChangeUI::setDartStatusData()
//{
//  char data_str[30] = { ' ' };
//  if (base_.dart_client_cmd_data_.dart_launch_opening_status == 1)
//  {
//    sprintf(data_str, "Dart Status: Close");
//    graph.setColor(rm_referee::GraphColor::YELLOW);
//  }
//  else if (base_.dart_client_cmd_data_.dart_launch_opening_status == 2)
//  {
//    sprintf(data_str, "Dart Status: Changing");
//    graph.setColor(rm_referee::GraphColor::ORANGE);
//  }
//  else if (base_.dart_client_cmd_data_.dart_launch_opening_status == 0)
//  {
//    sprintf(data_str, "Dart Open!");
//    graph.setColor(rm_referee::GraphColor::GREEN);
//  }
//  graph.setContent(data_str);
//  graph.setOperation(rm_referee::GraphOperation::UPDATE);
//}

}  // namespace rm_referee
