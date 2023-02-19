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
  void updateStepQueueStateData(const rm_msgs::StepQueueState ::ConstPtr data, const ros::Time& last_get_data_time);

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

}  // namespace rm_referee
