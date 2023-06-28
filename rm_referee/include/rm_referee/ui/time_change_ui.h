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
  explicit TimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                        std::vector<Graph>* graph_queue)
    : UiBase(rpc_value, base, graph_queue)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  void update() override;
  void updateForQueue();
  virtual void updateConfig(){};
};

class TimeChangeGroupUi : public GroupUiBase
{
public:
  explicit TimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                             std::vector<Graph>* graph_queue)
    : GroupUiBase(rpc_value, base, graph_queue)
  {
    graph_name_ = graph_name;
  }
  void update() override;
  void updateForQueue();
  virtual void updateConfig(){};

protected:
  std::string graph_name_;
};

class CapacitorTimeChangeUi : public TimeChangeUi
{
public:
  explicit CapacitorTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "capacitor", graph_queue){};
  void add() override;
  void updateRemainCharge(const double remain_charge, const ros::Time& time);

private:
  void updateConfig() override;
  double remain_charge_;
};

class EffortTimeChangeUi : public TimeChangeUi
{
public:
  explicit EffortTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "effort", graph_queue){};
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

private:
  void updateConfig() override;
  double joint_effort_;
  std::string joint_name_;
};

class ProgressTimeChangeUi : public TimeChangeUi
{
public:
  explicit ProgressTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "progress", graph_queue){};
  void updateEngineerUiData(const rm_msgs::EngineerUi::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void updateConfig() override;
  uint32_t finished_data_, total_steps_;
  std::string step_name_;
};

class DartStatusTimeChangeUi : public TimeChangeUi
{
public:
  explicit DartStatusTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "dart", graph_queue){};
  void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void updateConfig() override;
  uint8_t dart_launch_opening_status_;
};

class RotationTimeChangeUi : public TimeChangeUi
{
public:
  explicit RotationTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "rotation", graph_queue)
  {
    if (rpc_value.hasMember("data"))
    {
      XmlRpc::XmlRpcValue data = rpc_value["data"];
      try
      {
        arc_scale_ = static_cast<int>(data["scale"]);
        gimbal_reference_frame_ = static_cast<std::string>(data["gimbal_reference_frame"]);
        chassis_reference_frame_ = static_cast<std::string>(data["chassis_reference_frame"]);
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check configuration is exit");
      }
    }
    else
      ROS_WARN("RotationTimeChangeUi config 's member 'data' not defined.");
  };

private:
  void updateConfig() override;
  int arc_scale_;
  std::string gimbal_reference_frame_, chassis_reference_frame_;
};

class LaneLineTimeChangeGroupUi : public TimeChangeGroupUi
{
public:
  explicit LaneLineTimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeGroupUi(rpc_value, base, "lane_line", graph_queue)
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
      ROS_WARN("LaneLineTimeChangeGroupUi config 's member 'data' not defined.");

    if (rpc_value.hasMember("reference_joint"))
    {
      reference_joint_ = static_cast<std::string>(rpc_value["reference_joint"]);
    }
    else
      ROS_WARN("LaneLineTimeChangeGroupUi config 's member 'reference_joint' not defined.");

    graph_vector_.insert(
        std::pair<std::string, Graph*>(graph_name_ + "_left", new Graph(rpc_value["config"], base_, id_++)));
    graph_vector_.insert(
        std::pair<std::string, Graph*>(graph_name_ + "_right", new Graph(rpc_value["config"], base_, id_++)));

    for (auto it : graph_vector_)
      lane_line_double_graph_.push_back(it.second);
  }
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

protected:
  std::string reference_joint_;
  double robot_radius_, robot_height_, camera_range_, surface_coefficient_ = 0.5;
  double pitch_angle_ = 0., screen_x_ = 1920, screen_y_ = 1080;
  double end_point_a_angle_, end_point_b_angle_;

private:
  void updateConfig() override;

  std::vector<Graph*> lane_line_double_graph_;
};

class BalancePitchTimeChangeGroupUi : public TimeChangeGroupUi
{
public:
  explicit BalancePitchTimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeGroupUi(rpc_value, base, "balance_pitch", graph_queue)
  {
    XmlRpc::XmlRpcValue config;

    config["type"] = "line";
    if (rpc_value["config"].hasMember("color"))
      config["color"] = rpc_value["config"]["color"];
    else
      config["color"] = "cyan";
    if (rpc_value["config"].hasMember("width"))
      config["width"] = rpc_value["config"]["width"];
    else
      config["width"] = 2;
    if (rpc_value["config"].hasMember("delay"))
      config["delay"] = rpc_value["config"]["delay"];
    else
      config["delay"] = 0.2;

    XmlRpc::XmlRpcValue data = rpc_value["data"];
    ROS_ASSERT(data.hasMember("centre_point") && data.hasMember("bottom_angle") && data.hasMember("length"));
    centre_point_[0] = static_cast<int>(data["centre_point"][0]);
    centre_point_[1] = static_cast<int>(data["centre_point"][1]);
    bottom_angle_ = data["bottom_angle"];
    length_ = data["length"];
    triangle_left_point_[0] = centre_point_[0] - length_ * sin(bottom_angle_ / 2);
    triangle_left_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2);
    triangle_right_point_[0] = centre_point_[0] + length_ * sin(bottom_angle_ / 2);
    triangle_right_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2);

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

  void calculatePointPosition(const rm_msgs::BalanceStateConstPtr& data, const ros::Time& time);

private:
  void updateConfig() override;

  int centre_point_[2], triangle_left_point_[2], triangle_right_point_[2], length_;
  double bottom_angle_;
};

class PitchAngleTimeChangeUi : public TimeChangeUi
{
public:
  explicit PitchAngleTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : TimeChangeUi(rpc_value, base, "pitch", graph_queue){};
  void update() override;
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

private:
  void updateConfig() override;
  double pitch_angle_ = 0.;
};

class JointPositionTimeChangeUi : public TimeChangeUi
{
public:
  explicit JointPositionTimeChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue,
                                     std::string name)
    : TimeChangeUi(rpc_value, base, name, graph_queue)
  {
    if (rpc_value.hasMember("data"))
    {
      XmlRpc::XmlRpcValue data = rpc_value["data"];
      min_val_ = static_cast<double>(data["min_val"]);
      max_val_ = static_cast<double>(data["max_val"]);
      direction_ = static_cast<std::string>(data["direction"]);
      length_ = static_cast<double>(data["line_length"]);
    }
    name_ = name;
  };
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);

private:
  void updateConfig() override;
  std::string name_, direction_;
  double max_val_, min_val_, current_val_, length_;
};

class SpaceTfTimeChangeGroupUi : public TimeChangeGroupUi
{
public:
  struct Vector2D
  {
    int x;
    int y;
  };
  explicit SpaceTfTimeChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue,
                                    std::string name)
    : TimeChangeGroupUi(rpc_value, base, name, graph_queue)
  {
    tf_info_.resize(6, 0.);
    xyz_length_.resize(3, 0.);
    x_range_.resize(2, 0.);
    y_range_.resize(2, 0.);
    z_range_.resize(2, 0.);
    roll_range_.resize(2, 0.);
    pitch_range_.resize(2, 0);
    yaw_range_.resize(2, 0.);
    name_ = name;
    XmlRpc::XmlRpcValue config = rpc_value["config"];
    start_point_.x = static_cast<int>(config["start_position"][0]);
    start_point_.y = static_cast<int>(config["start_position"][1]);
    if (rpc_value.hasMember("tf"))
    {
      XmlRpc::XmlRpcValue tf = rpc_value["tf"];
      for (int i = 0; i < (int)xyz_length_.size(); ++i)
      {
        xyz_length_[i] = static_cast<int>(xmlRpcGetDouble(tf["xyz_length"], i));
      }
      for (int i = 0; i < (int)x_range_.size(); ++i)
      {
        x_range_[i] = xmlRpcGetDouble(tf["x_range"], i);
        y_range_[i] = xmlRpcGetDouble(tf["y_range"], i);
        z_range_[i] = xmlRpcGetDouble(tf["z_range"], i);
        roll_range_[i] = xmlRpcGetDouble(tf["roll_range"], i);
        pitch_range_[i] = xmlRpcGetDouble(tf["pitch_range"], i);
        yaw_range_[i] = xmlRpcGetDouble(tf["yaw_range"], i);
      }
      range_gather_.push_back(x_range_);
      range_gather_.push_back(y_range_);
      range_gather_.push_back(z_range_);
      range_gather_.push_back(roll_range_);
      range_gather_.push_back(pitch_range_);
      range_gather_.push_back(yaw_range_);
    }
    Vector2D temp_vec{ 0, 0 };
    end_points_.clear();
    temp_vec.x = start_point_.x;
    temp_vec.y = start_point_.y;
    end_points_.push_back(temp_vec);
    temp_vec.x = start_point_.x - xyz_length_[1];
    temp_vec.y = start_point_.y;
    end_points_.push_back(temp_vec);
    temp_vec.x = start_point_.x;
    temp_vec.y = start_point_.y + xyz_length_[2];
    end_points_.push_back(temp_vec);
    store_end_points_ = end_points_;
    vision_points_ = end_points_;

    XmlRpc::XmlRpcValue mimic_config;
    mimic_config["type"] = "line";
    if (rpc_value["config"].hasMember("width"))
      mimic_config["width"] = rpc_value["config"]["width"];
    else
      mimic_config["width"] = 2;
    if (rpc_value["config"].hasMember("delay"))
      mimic_config["delay"] = rpc_value["config"]["delay"];
    else
      mimic_config["delay"] = 0.2;

    mimic_config["start_position"][0] = start_point_.x;
    mimic_config["start_position"][1] = start_point_.y;

    mimic_config["end_position"][0] = end_points_[0].x;
    mimic_config["end_position"][1] = end_points_[0].y;
    graph_vector_.insert(std::make_pair<std::string, Graph*>("x", new Graph(mimic_config, base_, id_++)));
    mimic_config["end_position"][0] = end_points_[1].x;
    mimic_config["end_position"][1] = end_points_[1].y;
    graph_vector_.insert(std::make_pair<std::string, Graph*>("y", new Graph(mimic_config, base_, id_++)));
    mimic_config["end_position"][0] = end_points_[2].x;
    mimic_config["end_position"][1] = end_points_[2].y;
    graph_vector_.insert(std::make_pair<std::string, Graph*>("z", new Graph(mimic_config, base_, id_++)));
  };
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time);
  void calculateTransformedEndpoint(const Vector2D& start_point, std::vector<Vector2D>& end_points, double roll,
                                    double pitch, double yaw);

private:
  void updateConfig() override;
  std::string name_{};
  std::vector<double> tf_info_{};
  std::vector<int> xyz_length_{};
  std::vector<double> x_range_{}, y_range_{}, z_range_{}, roll_range_{}, pitch_range_{}, yaw_range_{};
  std::vector<std::vector<double>> range_gather_{};
  Vector2D start_point_{};
  std::vector<Vector2D> end_points_{}, vision_points_{}, store_end_points_{};
};
}  // namespace rm_referee
