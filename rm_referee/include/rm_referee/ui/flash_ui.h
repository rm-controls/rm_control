//
// Created by llljjjqqq on 22-11-4.
//

#pragma once

#include "rm_referee/ui/ui_base.h"

namespace rm_referee
{
class FlashUi : public UiBase
{
public:
  explicit FlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                   std::vector<Graph>* graph_queue)
    : UiBase(rpc_value, base, graph_queue)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void display(const ros::Time& time){};
  virtual void updateConfig(){};
};

class CoverFlashUi : public FlashUi
{
public:
  explicit CoverFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : FlashUi(rpc_value, base, "cover", graph_queue){};
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time) override;

private:
  void display(const ros::Time& time) override;
  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::vector<Graph>* graph_queue)
    : FlashUi(rpc_value, base, "spin", graph_queue){};
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void display(const ros::Time& time) override;
  uint8_t chassis_mode_;
};
}  // namespace rm_referee
