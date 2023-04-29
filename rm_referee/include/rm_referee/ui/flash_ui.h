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
  explicit FlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name) : UiBase(rpc_value, base)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void display(const ros::Time& time){};
  virtual void updateConfig(){};
};

class CoverFlashUi : public FlashUi
{
public:
  explicit CoverFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : FlashUi(rpc_value, base, "cover"){};
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time) override;

private:
  void display(const ros::Time& time) override;
  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : FlashUi(rpc_value, base, "spin"){};
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void display(const ros::Time& time) override;
  uint8_t chassis_mode_;
};
}  // namespace rm_referee
