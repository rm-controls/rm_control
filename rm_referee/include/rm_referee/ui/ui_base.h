//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/heat_limit.h>
#include <rm_msgs/StatusChangeRequest.h>

#include "rm_referee/ui/graph.h"

namespace rm_referee
{
class UiBase
{
public:
  explicit UiBase(Base& base) : base_(base), tf_listener_(tf_buffer_){};
  ~UiBase() = default;
  virtual void add();
  virtual void erasure();
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data){};
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time){};

protected:
  Base& base_;
  Graph* graph_;
  static int id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

class GroupUiBase : public UiBase
{
public:
  explicit GroupUiBase(XmlRpc::XmlRpcValue& rpc_value, Base& base) : UiBase(base)
  {
    ROS_ASSERT(rpc_value.hasMember("group_name"));
    group_name_ = static_cast<std::string>(rpc_value["group_name"]);
  };
  ~GroupUiBase() = default;
  virtual void add() override;
  virtual void erasure() override;
  void display();

protected:
  std::string group_name_;
  std::map<std::string, Graph*> graph_vector_;
};

class FixedUi : public UiBase
{
public:
  explicit FixedUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : UiBase(base)
  {
    for (int i = 0; i < static_cast<int>(rpc_value.size()); i++)
      graph_vector_.insert(
          std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
  };
  void add() override;
  void display();

  std::map<std::string, Graph*> graph_vector_;
};

class PolygonUi : public GroupUiBase
{
public:
  explicit PolygonUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : GroupUiBase(rpc_value, base)
  {
    ROS_ASSERT(rpc_value.hasMember("points"));
    XmlRpc::XmlRpcValue config(XmlRpc::XmlRpcValue::TypeStruct);

    config["type"] = "line";

    if (rpc_value["graph_config"].hasMember("color"))
      config["color"] = rpc_value["graph_config"]["color"];
    else
      config["color"] = "cyan";
    if (rpc_value["graph_config"].hasMember("width"))
      config["width"] = rpc_value["graph_config"]["width"];
    else
      config["width"] = 2;

    XmlRpc::XmlRpcValue points = rpc_value["points"];
    config["start_position"].setSize(2);
    config["end_position"].setSize(2);
    for (int i = 1; i <= points["points"].size(); i++)
    {
      if (i != points["points"].size())
      {
        config["start_position"][0] = points[i - 1][0];
        config["start_position"][1] = points[i - 1][1];
        config["end_position"][0] = points[i][0];
        config["end_position"][1] = points[i][1];
      }
      else
      {
        // Connect first and last
        config["start_position"][0] = points[i - 1][0];
        config["start_position"][1] = points[i - 1][1];
        config["end_position"][0] = points[0][0];
        config["end_position"][1] = points[0][1];
      }
      graph_vector_.insert(
          std::make_pair<std::string, Graph*>(group_name_ + "_" + std::to_string(i), new Graph(config, base_, id_++)));
    }
  }
};
}  // namespace rm_referee
