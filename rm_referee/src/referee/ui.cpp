//
// Created by peter on 2021/7/20.
//

#include "rm_referee/referee/ui.h"

namespace rm_referee
{
int UiBase::id_(2);
UiBase::UiBase(ros::NodeHandle& nh, Data& data, const std::string& ui_type) : data_(data)
{
  XmlRpc::XmlRpcValue rpc_value;
  if (!nh.getParam(ui_type, rpc_value))
  {
    ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
    return;
  }
  try
  {
    for (int i = 0; i < (int)rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "chassis")
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], data_.base_, 1)));
      else
        graph_vector_.insert(std::pair<std::string, Graph*>(rpc_value[i]["name"],
                                                            new Graph(rpc_value[i]["config"], data_.base_, id_++)));
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("Wrong ui parameter: %s", e.getMessage().c_str());
  }
  for (auto graph : graph_vector_)
    graph.second->setOperation(rm_common::GraphOperation::DELETE);
}

void UiBase::add()
{
  for (auto graph : graph_vector_)
  {
    graph.second->setOperation(rm_common::GraphOperation::ADD);
    graph.second->display(true);
    graph.second->sendUi(ros::Time::now());
  }
}

TriggerChangeUi::TriggerChangeUi(ros::NodeHandle& nh, Data& data) : UiBase(nh, data, "trigger_change")
{
  for (auto graph : graph_vector_)
  {
    if (graph.first == "chassis")
    {
      if (data_.base_.robot_id_ == rm_common::RobotId::RED_ENGINEER ||
          data_.base_.robot_id_ == rm_common::RobotId::BLUE_ENGINEER)
        graph.second->setContent("raw");
      else
        graph.second->setContent("follow");
    }
    else if (graph.first == "target")
    {
      graph.second->setContent("armor");
      if (data_.base_.robot_color_ == "red")
        graph.second->setColor(rm_common::GraphColor::CYAN);
      else
        graph.second->setColor(rm_common::GraphColor::PINK);
    }
    else
      graph.second->setContent("0");
  }
}

void TriggerChangeUi::update(const std::string& graph_name, const std::string& content)
{
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end())
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
    graph->second->setOperation(rm_common::GraphOperation::UPDATE);
    graph->second->display();
    graph->second->sendUi(ros::Time::now());
  }
}

void TriggerChangeUi::update(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode,
                             bool sub_flag)
{
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end())
  {
    updateConfig(graph_name, graph->second, main_mode, main_flag, sub_mode, sub_flag);
    graph->second->setOperation(rm_common::GraphOperation::UPDATE);
    if (graph->first == "chassis" || graph->first == "gimbal")
    {
      graph->second->displayTwice(true);
      graph->second->sendUi(ros::Time::now());
    }
    else
    {
      graph->second->display();
      graph->second->sendUi(ros::Time::now());
    }
  }
}

void TriggerChangeUi::updateConfig(const std::string& name, Graph* graph, uint8_t main_mode, bool main_flag,
                                   uint8_t sub_mode, bool sub_flag)
{
  if (name == "chassis")
  {
    if (main_mode == 254)
    {
      graph->setContent("Cap reset");
      graph->setColor(rm_common::GraphColor::YELLOW);
      return;
    }
    graph->setContent(getChassisState(main_mode));
    if (main_flag)
      graph->setColor(rm_common::GraphColor::ORANGE);
    else if (sub_flag)
      graph->setColor(rm_common::GraphColor::GREEN);
    else if (sub_mode == 1)
      graph->setColor(rm_common::GraphColor::PINK);
    else
      graph->setColor(rm_common::GraphColor::WHITE);
  }
  else if (name == "shooter")
  {
    graph->setContent(getShooterState(main_mode));
    if (sub_mode == rm_common::HeatLimit::LOW)
      graph->setColor(rm_common::GraphColor::WHITE);
    else if (sub_mode == rm_common::HeatLimit::HIGH)
      graph->setColor(rm_common::GraphColor::YELLOW);
    else if (sub_mode == rm_common::HeatLimit::BURST)
      graph->setColor(rm_common::GraphColor::ORANGE);
  }
  else if (name == "gimbal")
  {
    graph->setContent(getGimbalState(main_mode));
    if (main_flag)
      graph->setColor(rm_common::GraphColor::ORANGE);
    else
      graph->setColor(rm_common::GraphColor::WHITE);
  }
  else if (name == "target")
  {
    graph->setContent(getTargetState(main_mode, sub_mode));
    if (main_flag)
      graph->setColor(rm_common::GraphColor::ORANGE);
    else if (sub_flag)
      graph->setColor(rm_common::GraphColor::PINK);
    else
      graph->setColor(rm_common::GraphColor::CYAN);
  }
  else if (name == "card")
  {
    if (main_flag)
      graph->setContent("on");
    else
      graph->setContent("off");
  }
  else if (name == "sentry")
  {
    if (main_mode == 0)
      graph->setContent("stop");
    else
      graph->setContent("move");
  }
}

std::string TriggerChangeUi::getShooterState(uint8_t mode)
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

std::string TriggerChangeUi::getChassisState(uint8_t mode)
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

std::string TriggerChangeUi::getTargetState(uint8_t target, uint8_t armor_target)
{
  if (data_.base_.robot_id_ != rm_common::RobotId::BLUE_HERO && data_.base_.robot_id_ != rm_common::RobotId::RED_HERO)
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

std::string TriggerChangeUi::getGimbalState(uint8_t mode)
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

void FixedUi::update()
{
  for (auto graph : graph_vector_)
  {
    graph.second->updatePosition(getShootSpeedIndex());
    graph.second->setOperation(rm_common::GraphOperation::UPDATE);
    graph.second->display();
    graph.second->sendUi(ros::Time::now());
  }
}

int FixedUi::getShootSpeedIndex()
{
  uint16_t speed_limit;
  if (data_.base_.robot_id_ != rm_common::RobotId::BLUE_HERO && data_.base_.robot_id_ != rm_common::RobotId::RED_HERO)
  {
    speed_limit = data_.base_.game_robot_status_data_.shooter_id_1_17_mm_speed_limit;
    if (speed_limit == 15)
      return 0;
    else if (speed_limit == 18)
      return 1;
    else if (speed_limit == 30)
      return 2;
  }
  return 0;
}

void FlashUi::update(const std::string& name, const ros::Time& time, bool state)
{
  auto graph = graph_vector_.find(name);
  if (graph == graph_vector_.end())
    return;
  if (name.find("armor") != std::string::npos)
  {
    if (data_.base_.robot_hurt_data_.hurt_type == 0x00 &&
        data_.base_.robot_hurt_data_.armor_id == getArmorId(graph->first))
    {
      updateArmorPosition(graph->first, graph->second);
      graph->second->display(time, true, true);
      graph->second->sendUi(time);
      data_.base_.robot_hurt_data_.hurt_type = 9;
    }
    else
    {
      graph->second->display(time, false, true);
      graph->second->sendUi(time);
    }
  }
  else
  {
    if (name == "aux")
      updateChassisGimbalDate(data_.joint_state_.position[8], graph->second);
    if (state)
      graph->second->setOperation(rm_common::GraphOperation::DELETE);

    if (name == "cover")
      graph->second->display(time, !state, true);
    else
      graph->second->display(time, !state);
    graph->second->sendUi(time);
  }
}

void FlashUi::updateChassisGimbalDate(const double yaw_joint_, Graph* graph)
{
  double cover_yaw_joint = yaw_joint_;
  while (abs(cover_yaw_joint) > 2 * M_PI)
  {
    cover_yaw_joint += cover_yaw_joint > 0 ? -2 * M_PI : 2 * M_PI;
  }
  graph->setStartX(960 - 50 * sin(cover_yaw_joint));
  graph->setStartY(540 + 50 * cos(cover_yaw_joint));

  graph->setEndX(960 - 100 * sin(cover_yaw_joint));
  graph->setEndY(540 + 100 * cos(cover_yaw_joint));
}

void FlashUi::updateArmorPosition(const std::string& name, Graph* graph)
{
  geometry_msgs::TransformStamped yaw_2_baselink;
  double roll, pitch, yaw;
  try
  {
    yaw_2_baselink = data_.tf_buffer_.lookupTransform("yaw", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
  }
  quatToRPY(yaw_2_baselink.transform.rotation, roll, pitch, yaw);
  if (getArmorId(name) == 0 || getArmorId(name) == 2)
  {
    graph->setStartX((int)(960 + 340 * sin(getArmorId(name) * M_PI_2 + yaw)));
    graph->setStartY((int)(540 + 340 * cos(getArmorId(name) * M_PI_2 + yaw)));
  }
  else
  {
    graph->setStartX((int)(960 + 340 * sin(-getArmorId(name) * M_PI_2 + yaw)));
    graph->setStartY((int)(540 + 340 * cos(-getArmorId(name) * M_PI_2 + yaw)));
  }
}

uint8_t FlashUi::getArmorId(const std::string& name)
{
  if (name == "armor0")
    return 0;
  else if (name == "armor1")
    return 1;
  else if (name == "armor2")
    return 2;
  else if (name == "armor3")
    return 3;
  return 9;
}

void TimeChangeUi::add()
{
  for (auto graph : graph_vector_)
  {
    if (graph.first == "capacitor" && data_.base_.capacity_data_.cap_power == 0.)
      continue;
    graph.second->setOperation(rm_common::GraphOperation::ADD);
    graph.second->display(true);
    graph.second->sendUi(ros::Time::now());
  }
}

void TimeChangeUi::update(const std::string& name, const ros::Time& time, double data)
{
  auto graph = graph_vector_.find(name);
  if (graph != graph_vector_.end())
  {
    if (name == "capacitor")
      setCapacitorData(*graph->second);
    if (name == "effort")
      setEffortData(*graph->second);
    if (name == "progress")
      setProgressData(*graph->second, data);
    if (name == "temperature")
      setTemperatureData(*graph->second);
    if (name == "dart_status")
      setDartStatusData(*graph->second);
    graph->second->display(time);
    graph->second->sendUi(ros::Time::now());
  }
}

void TimeChangeUi::setOreRemindData(Graph& graph)
{
  char data_str[30] = { ' ' };
  int time = data_.base_.game_status_data_.stage_remain_time;
  if (time < 420 && time > 417)
    sprintf(data_str, "Ore will released after 15s");
  else if (time < 272 && time > 269)
    sprintf(data_str, "Ore will released after 30s");
  else if (time < 252 && time > 249)
    sprintf(data_str, "Ore will released after 10s");
  else
    return;
  graph.setContent(data_str);
  graph.setOperation(rm_common::GraphOperation::UPDATE);
}

void TimeChangeUi::setDartStatusData(Graph& graph)
{
  char data_str[30] = { ' ' };
  if (data_.base_.dart_client_cmd_data_.dart_launch_opening_status == 1)
  {
    sprintf(data_str, "Dart Status: Close");
    graph.setColor(rm_common::GraphColor::YELLOW);
  }
  else if (data_.base_.dart_client_cmd_data_.dart_launch_opening_status == 2)
  {
    sprintf(data_str, "Dart Status: Changing");
    graph.setColor(rm_common::GraphColor::ORANGE);
  }
  else if (data_.base_.dart_client_cmd_data_.dart_launch_opening_status == 0)
  {
    sprintf(data_str, "Dart Open!");
    graph.setColor(rm_common::GraphColor::GREEN);
  }
  graph.setContent(data_str);
  graph.setOperation(rm_common::GraphOperation::UPDATE);
}

void TimeChangeUi::setCapacitorData(Graph& graph)
{
  if (data_.base_.capacity_data_.cap_power != 0.)
  {
    char data_str[30] = { ' ' };
    double cap_power = data_.base_.capacity_data_.cap_power * 100.;
    if (cap_power > 0.)
      sprintf(data_str, "cap:%1.0f%%", cap_power);
    else
      sprintf(data_str, "please charge");
    graph.setContent(data_str);
    if (cap_power < 30.)
      graph.setColor(rm_common::GraphColor::ORANGE);
    else if (cap_power > 70.)
      graph.setColor(rm_common::GraphColor::GREEN);
    else
      graph.setColor(rm_common::GraphColor::YELLOW);
    graph.setOperation(rm_common::GraphOperation::UPDATE);
  }
}

void TimeChangeUi::setEffortData(Graph& graph)
{
  char data_str[30] = { ' ' };
  int max_index = 0;
  if (!data_.joint_state_.name.empty())
  {
    for (int i = 0; i < (int)data_.joint_state_.effort.size(); ++i)
      if ((data_.joint_state_.name[i] == "joint1" || data_.joint_state_.name[i] == "joint2" ||
           data_.joint_state_.name[i] == "joint3" || data_.joint_state_.name[i] == "joint4" ||
           data_.joint_state_.name[i] == "joint5") &&
          data_.joint_state_.effort[i] > data_.joint_state_.effort[max_index])
        max_index = i;
    if (max_index != 0)
    {
      sprintf(data_str, "%s:%.2f N.m", data_.joint_state_.name[max_index].c_str(), data_.joint_state_.effort[max_index]);
      graph.setContent(data_str);
      if (data_.joint_state_.effort[max_index] > 20.)
        graph.setColor(rm_common::GraphColor::ORANGE);
      else if (data_.joint_state_.effort[max_index] < 10.)
        graph.setColor(rm_common::GraphColor::GREEN);
      else
        graph.setColor(rm_common::GraphColor::YELLOW);
      graph.setOperation(rm_common::GraphOperation::UPDATE);
    }
  }
}

void TimeChangeUi::setProgressData(Graph& graph, double data)
{
  char data_str[30] = { ' ' };
  sprintf(data_str, " %.1f%%", data * 100.);
  graph.setContent(data_str);
  graph.setOperation(rm_common::GraphOperation::UPDATE);
}

void TimeChangeUi::setTemperatureData(Graph& graph)
{
  char data_str[30] = { ' ' };
  for (int i = 0; i < (int)data_.actuator_state_.name.size(); ++i)
  {
    if (data_.actuator_state_.name[i] == "right_finger_joint_motor")
    {
      sprintf(data_str, " %.1hhu C", data_.actuator_state_.temperature[i]);
      graph.setContent(data_str);
      if (data_.actuator_state_.temperature[i] > 70.)
        graph.setColor(rm_common::GraphColor::ORANGE);
      else if (data_.actuator_state_.temperature[i] < 30.)
        graph.setColor(rm_common::GraphColor::GREEN);
      else
        graph.setColor(rm_common::GraphColor::YELLOW);
    }
  }
  graph.setOperation(rm_common::GraphOperation::UPDATE);
}

}  // namespace rm_referee
