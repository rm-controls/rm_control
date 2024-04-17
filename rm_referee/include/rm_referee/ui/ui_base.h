//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/heat_limit.h>
#include <rm_msgs/StatusChangeRequest.h>

#include "rm_referee/ui/graph.h"

namespace rm_referee
{
class UiBase
{
public:
  explicit UiBase(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                  std::deque<Graph>* character_queue = nullptr)
    : base_(base), tf_listener_(tf_buffer_)
  {
    if (rpc_value.hasMember("config"))
      if (rpc_value["config"].hasMember("delay"))
        delay_ = ros::Duration(static_cast<double>(rpc_value["config"]["delay"]));
    graph_queue_ = graph_queue;
    character_queue_ = character_queue;
  };
  ~UiBase() = default;
  virtual void add();
  virtual void update();
  virtual void updateForQueue();
  virtual void erasure();
  virtual void addForQueue(int add_times = 1);
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data){};
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time){};
  virtual void sendUi(const ros::Time& time);

  void sendCharacter(const ros::Time& time, Graph* graph);
  void sendSingleGraph(const ros::Time& time, Graph* graph);
  void transferInt(const int data);

  void sendSerial(const ros::Time& time, int data_len);
  void clearTxBuffer();

  virtual void display(bool check_repeat = true);
  virtual void displayTwice(bool check_repeat = true);
  virtual void display(const ros::Time& time);
  void display(const ros::Time& time, bool state, bool once = false);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const;

  uint8_t tx_buffer_[128];
  int tx_len_;

protected:
  Base& base_;
  Graph* graph_;
  static int id_;
  std::deque<Graph>* graph_queue_;
  std::deque<Graph>* character_queue_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Time last_send_;
  ros::Duration delay_ = ros::Duration(0.);
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

class InteractiveSender : public UiBase
{
public:
  explicit InteractiveSender(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                             std::deque<Graph>* character_queue = nullptr)
    : UiBase(rpc_value, base, graph_queue, character_queue){};

  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void sendRadarInteractiveData(const rm_referee::ClientMapReceiveData& data);
  void sendMapSentryData(const rm_referee::MapSentryData& data);
  void sendCurrentSentryData(const rm_msgs::CurrentSentryPosDataConstPtr& data);
  void sendCustomInfoData(std::wstring data);
  void sendSentryCmdData(const rm_msgs::SentryInfoConstPtr& data);
  void sendRadarCmdData(const rm_msgs::RadarInfoConstPtr& data);

protected:
  std::wstring last_custom_info_;
};

class GroupUiBase : public UiBase
{
public:
  explicit GroupUiBase(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                       std::deque<Graph>* character_queue = nullptr)
    : UiBase(rpc_value, base, graph_queue, character_queue){};
  ~GroupUiBase() = default;
  void add() override;
  void update() override;
  void updateForQueue() override;
  void erasure() override;
  void addForQueue(int add_times = 1) override;
  void sendUi(const ros::Time& time) override;
  void sendDoubleGraph(const ros::Time& time, Graph* graph0, Graph* graph1);
  void sendFiveGraph(const ros::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3, Graph* graph4);
  void sendSevenGraph(const ros::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3, Graph* graph4,
                      Graph* graph5, Graph* graph6);
  void display(bool check_repeat = true) override;
  void display(const ros::Time& time) override;
  void displayTwice(bool check_repeat = true) override;

protected:
  std::map<std::string, Graph*> graph_vector_;
  std::map<std::string, Graph*> character_vector_;
};

class FixedUi : public GroupUiBase
{
public:
  explicit FixedUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                   std::deque<Graph>* character_queue = nullptr)
    : GroupUiBase(rpc_value, base, graph_queue, character_queue)
  {
    for (int i = 0; i < static_cast<int>(rpc_value.size()); i++)
    {
      if (rpc_value[i]["config"]["type"] == "string")
      {
        ROS_INFO_STREAM("string FixedUi:" << rpc_value[i]["name"]);
        character_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
      }
      else
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
    }
  };
  void updateForQueue() override;
  int update_fixed_ui_times = 0;
};

}  // namespace rm_referee
