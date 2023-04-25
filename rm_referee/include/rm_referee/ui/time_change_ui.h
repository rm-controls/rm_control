//
// Created by llljjjqqq on 22-11-4.
//

#pragma once

#include "rm_referee/ui/ui_base.h"

namespace rm_referee
{
class TimeChangeUi : public UiBase
{
public:
  explicit TimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name) : UiBase(base)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void display(const ros::Time& time);
  virtual void updateConfig(){};
};

class TimeChangeGroupUi : public GroupUiBase
{
public:
  explicit TimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name)
    : GroupUiBase(base)
  {
  }
  virtual void display(const ros::Time& time);
  virtual void updateConfig(){};
};

class CapacitorTimeChangeUi : public TimeChangeUi
{
public:
  explicit CapacitorTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TimeChangeUi(rpc_value, base, "capacitor"){};
  void add() override;
  void updateCapacityData(const rm_msgs::CapacityData data, const ros::Time& time);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  double cap_power_;
};

class EffortTimeChangeUi : public TimeChangeUi
{
public:
  explicit EffortTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TimeChangeUi(rpc_value, base, "effort"){};
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  double joint_effort_;
  std::string joint_name_;
};

class ProgressTimeChangeUi : public TimeChangeUi
{
public:
  explicit ProgressTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TimeChangeUi(rpc_value, base, "progress"){};
  void updateEngineerUiData(const rm_msgs::EngineerUi::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  uint32_t finished_data_, total_steps_;
  std::string step_name_;
};

class DartStatusTimeChangeUi : public TimeChangeUi
{
public:
  explicit DartStatusTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TimeChangeUi(rpc_value, base, "dart"){};
  void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  uint8_t dart_launch_opening_status_;
};

class LaneLineTimeChangeUi : public TimeChangeUi
{
public:
  explicit LaneLineTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TimeChangeUi(rpc_value, base, "lane_line")
  {
    if (rpc_value.hasMember("data"))
    {
      XmlRpc::XmlRpcValue& data = rpc_value["data"];
      robot_radius_ = data["radius"];
      robot_height_ = data["height"];
      camera_range_ = data["camera_range"];
      surface_coefficient_ = data["surface_coefficient"];
    }
    else
      ROS_WARN("LaneLineUi config 's member 'data' not defined.");

    if (rpc_value.hasMember("reference_joint"))
    {
      reference_joint_ = static_cast<std::string>(rpc_value["reference_joint"]);
    }
    else
      ROS_WARN("LaneLineUi config 's member 'reference_joint' not defined.");

    graph_left_ = UiBase::graph_;
    graph_right_ = new Graph(rpc_value["config"], base_, UiBase::id_++);
  }
  void add() override;
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

protected:
  Graph *graph_left_, *graph_right_;
  std::string reference_joint_;
  double robot_radius_, robot_height_, camera_range_, surface_coefficient_ = 0.5;
  double pitch_angle_ = 0., screen_x_ = 1920, screen_y_ = 1080;
  double end_point_a_angle_, end_point_b_angle_;

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
};

class BalancePitchTimeChangeGroupUi : public TimeChangeGroupUi
{
public:
  explicit BalancePitchTimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TimeChangeGroupUi(rpc_value, base, "balance_pitch")
  {
    XmlRpc::XmlRpcValue config;

    config["type"] = "line";
    if (rpc_value["graph_config"].hasMember("color"))
      config["color"] = rpc_value["graph_config"]["color"];
    else
      config["color"] = "cyan";
    if (rpc_value["graph_config"].hasMember("width"))
      config["width"] = rpc_value["graph_config"]["width"];
    else
      config["width"] = 2;
    if (rpc_value["graph_config"].hasMember("delay"))
      config["delay"] = rpc_value["graph_config"]["delay"];
    else
      config["delay"] = 0.2;

    ROS_ASSERT(rpc_value["graph_config"].hasMember("centre_point") &&
               rpc_value["graph_config"].hasMember("bottom_angle") && rpc_value["graph_config"].hasMember("length"));
    centre_point_[0] = static_cast<int>(rpc_value["graph_config"]["centre_point"][0]);
    centre_point_[1] = static_cast<int>(rpc_value["graph_config"]["centre_point"][1]);
    bottom_angle_ = rpc_value["graph_config"]["bottom_angle"];
    length_ = rpc_value["graph_config"]["length"];
    triangle_left_point_[0] = centre_point_[0] - length_ * sin(bottom_angle_ / 2);
    triangle_left_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2);
    triangle_right_point_[0] = centre_point_[0] + length_ * sin(bottom_angle_ / 2);
    triangle_right_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2);

    XmlRpc::XmlRpcValue config_factor;
    config["start_position"][0] = centre_point_[0] - length_;
    config["start_position"][1] = centre_point_[1];
    config["end_position"][0] = centre_point_[0] + length_;
    config["end_position"][1] = centre_point_[1];
    graph_vector_.insert(std::make_pair<std::string, Graph*>("bottom", new Graph(config, base_, id_++)));

    config["start_position"][0] = centre_point_[0];
    config["start_position"][1] = centre_point_[1];
    config["end_position"][0] = triangle_left_point_[0];
    config["end_position"][1] = triangle_left_point_[1];
    graph_vector_.insert(std::make_pair<std::string, Graph*>("triangle_left_side", new Graph(config, base_, id_++)));

    config["start_position"][0] = centre_point_[0];
    config["start_position"][1] = centre_point_[1];
    config["end_position"][0] = triangle_right_point_[0];
    config["end_position"][1] = triangle_right_point_[1];
    graph_vector_.insert(std::make_pair<std::string, Graph*>("triangle_right_side", new Graph(config, base_, id_++)));
  }

  void updateConfig() override;
  void display(const ros::Time& time) override;
  void calculatePointPosition(const rm_msgs::BalanceStateConstPtr& data, const ros::Time& time);

private:
  int centre_point_[2], triangle_left_point_[2], triangle_right_point_[2], length_;
  double bottom_angle_;
};

}  // namespace rm_referee
