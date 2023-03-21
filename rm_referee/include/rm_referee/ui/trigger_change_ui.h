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

class BloodVolumeTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit BloodVolumeTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "blood_volume"){

    };
  void add() override;
  std::string getRobotName(uint8_t id);
  int getRobotHp(uint8_t id);
  void updateRobotHpData(const rm_msgs::GameRobotHp data)
  {
    robot_hp_ = data;
  }
  void updateConfig(const rm_msgs::TrackData::ConstPtr data, const ros::Time& time);

private:
  void updateConfig(uint8_t robot_id, bool is_red, uint8_t sub_mode = 0, bool sub_flag = false) override;
  bool is_deleted_;
  rm_msgs::GameRobotHp robot_hp_;
  std::map<int, std::string> robot_name_vector_ = {
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_ENGINEER, "RED_ENGINEER"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_SENTRY, "RED_SENTRY"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_HERO, "RED_HERO"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_STANDARD_3, "RED_STANDARD3"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_STANDARD_4, "RED_STANDARD4"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::RED_STANDARD_5, "RED_STANDARD5"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_ENGINEER, "BLUE_ENGINEER"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_SENTRY, "BLUE_SENTRY"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_HERO, "BLUE_HERO"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_STANDARD_3, "BLUE_STANDARD3"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_STANDARD_4, "BLUE_STANDARD4"),
    std::make_pair<int, std::string>(rm_msgs::GameRobotStatus::BLUE_STANDARD_5, "BLUE_STANDARD5")

  };
  std::map<int, uint16_t*> robot_hp_vector_ = {
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_ENGINEER, &robot_hp_.red_2_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_SENTRY, &robot_hp_.red_7_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_HERO, &robot_hp_.red_1_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_STANDARD_3, &robot_hp_.red_3_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_STANDARD_4, &robot_hp_.red_4_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::RED_STANDARD_5, &robot_hp_.red_5_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_ENGINEER, &robot_hp_.blue_2_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_SENTRY, &robot_hp_.blue_7_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_HERO, &robot_hp_.blue_1_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_STANDARD_3, &robot_hp_.blue_3_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_STANDARD_4, &robot_hp_.blue_4_robot_hp),
    std::make_pair<int, uint16_t*>(rm_msgs::GameRobotStatus::BLUE_STANDARD_5, &robot_hp_.blue_5_robot_hp)
  };
};
}  // namespace rm_referee
