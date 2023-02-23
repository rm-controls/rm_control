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
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data){};
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time){};

protected:
  Base& base_;
  Graph* graph_;
  static int id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
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
}  // namespace rm_referee
