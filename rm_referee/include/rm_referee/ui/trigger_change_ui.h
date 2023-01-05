//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include "rm_referee/ui/ui_base.h"
#include <rm_common/decision/power_limit.h>

namespace rm_referee
{
class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name) : UiBase(base)
  {
    if (graph_name == "chassis")
      graph_ = new Graph(rpc_value["config"], base_, 1);
    else
      graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void setContent(const std::string& content);
  virtual void display();
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
};
class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "chassis")
  {
    if (base.robot_id_ == rm_referee::RobotId::RED_ENGINEER || base.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
      graph_->setContent("raw");
    else
      graph_->setContent("follow");
  }
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;
  void updateDbusData(const rm_msgs::DbusData::ConstPtr data);
  void updateCapacityData(const rm_msgs::CapacityData data);

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  void displayInCapacity();
  std::string getChassisState(uint8_t mode);
  uint8_t chassis_mode_, power_limit_state_, s_l_, s_r_, key_ctrl_, key_shift_, key_b_;
};

class ShooterTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ShooterTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "shooter")
  {
    graph_->setContent("0");
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getShooterState(uint8_t mode);
  uint8_t shooter_mode_, shoot_frequency_;
};

class GimbalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit GimbalTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "gimbal")
  {
    graph_->setContent("0");
  }
  void updateGimbalCmdData(const rm_msgs::GimbalCmd ::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getGimbalState(uint8_t mode);
  uint8_t gimbal_mode_, gimbal_eject_;
};

class TargetTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "target")
  {
    graph_->setContent("armor");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getTargetState(uint8_t target, uint8_t armor_target);
  uint8_t det_target_, shoot_frequency_, det_armor_target_, det_color_, gimbal_eject_;
};

class StepTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit StepTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TriggerChangeUi(rpc_value, base, "step")
  {
    graph_->setContent("step_name");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateStepUiData(const rm_msgs::EngineerUi ::ConstPtr data);

private:
  void display() override;
  void stepUpdateConfig(std::string step_name);
  std::string getStepName(std::string step_name);
  std::string step_name_;
};

class DragTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit DragTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TriggerChangeUi(rpc_value, base, "drag")
  {
    graph_->setContent("drag:");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateDragUiData(const rm_msgs::EngineerUi ::ConstPtr data);

private:
  void display() override;
  void dragUpdateConfig(std::string drag_state);
  std::string getDragState(std::string drag_state);
  std::string drag_state_;
};

class ReversalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ReversalTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "reversal")
  {
    graph_->setContent("reversal:");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateReversalUiData(const rm_msgs::EngineerUi ::ConstPtr data);

private:
  void display() override;
  void reversalUpdateConfig(std::string reversal_state);
  std::string getReversalState(std::string reversal_state);
  std::string reversal_state_;
};

class StoneTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit StoneTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base) : TriggerChangeUi(rpc_value, base, "stone")
  {
    graph_->setContent("stone:");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateStoneUiData(const rm_msgs::EngineerUi ::ConstPtr data);

private:
  void display() override;
  void stoneUpdateConfig(uint8_t stone_num);
  std::string getStoneNum(uint8_t stone_num);
  uint8_t stone_num_;
};
}  // namespace rm_referee
