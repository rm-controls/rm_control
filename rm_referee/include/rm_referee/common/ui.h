//
// Created by peter on 2021/5/24.
//

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/heat_limit.h>
#include <rm_msgs/StatusChangeRequest.h>
#include "rm_referee/common/graph.h"

namespace rm_referee
{
// UiBase
class UiBase
{
public:
  explicit UiBase(ros::NodeHandle& nh, Base& base, const std::string& ui_type);
  virtual void add();

protected:
  Base& base_;
  Graph* graph_;
  static int id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, Graph*> graph_vector_;
};

// TriggerChangeUi
class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name)
    : UiBase(nh, base, "trigger_change")
  {
    for (auto graph : graph_vector_)
      if (graph.first == graph_name)
        graph_ = graph.second;
  }
  virtual void setContent(const std::string& content);
  virtual void display();
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
};
class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "chassis")
  {
    if (base.robot_id_ == rm_referee::RobotId::RED_ENGINEER || base.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
      graph_->setContent("raw");
    else
      graph_->setContent("follow");
  }
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data);
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
  explicit ShooterTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "shooter")
  {
    graph_->setContent("0");
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data);

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getShooterState(uint8_t mode);
  uint8_t shooter_mode_, shoot_frequency_;
};

class GimbalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit GimbalTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "gimbal")
  {
    graph_->setContent("0");
  }
  void updateGimbalCmdData(const rm_msgs::GimbalCmd ::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data);

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getGimbalState(uint8_t mode);
  uint8_t gimbal_mode_, gimbal_eject_;
};

class TargetTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetTriggerChangeUi(ros::NodeHandle& nh, Base& base) : TriggerChangeUi(nh, base, "target")
  {
    for (auto graph : graph_vector_)
      graph_->setContent("armor");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data);

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getTargetState(uint8_t target, uint8_t armor_target);
  uint8_t det_target_, shoot_frequency_, det_armor_target_, det_color_, gimbal_eject_;
};

//// FixedUi
class FixedUi : public UiBase
{
public:
  explicit FixedUi(ros::NodeHandle& nh, Base& base) : UiBase(nh, base, "fixed"){};
  void add() override;
  void display();
};

//// TimeChangeUi
class TimeChangeUi : public UiBase
{
public:
  explicit TimeChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name)
    : UiBase(nh, base, "time_change")
  {
    for (auto graph : graph_vector_)
      if (graph.first == graph_name)
        graph_ = graph.second;
  }
  virtual void display(const ros::Time& time);
  virtual void updateConfig(){};
};

class CapacitorTimeChangeUi : public TimeChangeUi
{
public:
  explicit CapacitorTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "capacitor"){};
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
  explicit EffortTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "effort"){};
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
  explicit ProgressTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "progress"){};
  void updateEngineerCmdData(const rm_msgs::EngineerCmd ::ConstPtr data, const ros::Time& last_get_);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  uint32_t finished_data_, total_steps_;
  std::string step_name_;
};

class DartStatusTimeChangeUi : public TimeChangeUi
{
public:
  explicit DartStatusTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "dart"){};
  void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data, const ros::Time& last_get_);

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  uint8_t dart_launch_opening_status_;
};

//// FlashUi
class FlashUi : public UiBase
{
public:
  explicit FlashUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name) : UiBase(nh, base, "flash")
  {
    for (auto graph : graph_vector_)
      if (graph.first == graph_name)
        graph_ = graph.second;
  }
  virtual void display(const ros::Time& time){};
  virtual void updateConfig(){};

private:
};

class ArmorFlashUi : public FlashUi
{
public:
  explicit ArmorFlashUi(ros::NodeHandle& nh, Base& base, std::string graph_name) : FlashUi(nh, base, graph_name)
  {
    graph_name_ = graph_name;
  };
  void updateRobotHurtData(const rm_msgs::RobotHurt data, const ros::Time& last_get_);

private:
  void display(const ros::Time& time) override;
  void updateArmorPosition();
  uint8_t getArmorId();

  std::string graph_name_;
  uint8_t hurt_type_, armor_id_;
};

class CoverFlashUi : public FlashUi
{
public:
  explicit CoverFlashUi(ros::NodeHandle& nh, Base& base) : FlashUi(nh, base, "cover"){};
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_);

private:
  void display(const ros::Time& time) override;
  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(ros::NodeHandle& nh, Base& base) : FlashUi(nh, base, "spin"){};
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_);

private:
  void display(const ros::Time& time) override;
  uint8_t chassis_mode_;
};
}  // namespace rm_referee
