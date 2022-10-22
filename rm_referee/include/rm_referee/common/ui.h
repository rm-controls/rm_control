//
// Created by peter on 2021/5/24.
//

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/heat_limit.h>
#include "rm_referee/common/graph.h"
#include <rm_msgs/StatusChangeRequest.h>

namespace rm_referee
{
// UiBase
class UiBase
{
public:
  explicit UiBase(ros::NodeHandle& nh, Base& base, const std::string& ui_type);
  virtual void add();
  virtual void updateStatusData(const rm_msgs::GameRobotStatus data){};
  virtual void updateStatusData(const rm_msgs::GameRobotStatus data, const ros::Time& last_get_){};
  virtual void updateGameStatusData(const rm_msgs::GameStatus data){};
  virtual void updateGameStatusData(const rm_msgs::GameStatus data, const ros::Time& last_get_){};
  virtual void updateCapacityData(const rm_msgs::CapacityData data){};
  virtual void updateCapacityData(const rm_msgs::CapacityData data, const ros::Time& last_get_){};
  virtual void updatePowerHeatData(const rm_msgs::PowerHeatData data){};
  virtual void updatePowerHeatData(const rm_msgs::PowerHeatData data, const ros::Time& last_get_){};
  virtual void updateRobotHurtData(const rm_msgs::RobotHurt data){};
  virtual void updateRobotHurtData(const rm_msgs::RobotHurt data, const ros::Time& last_get_){};
  virtual void updateInteractiveData(const rm_referee::InteractiveData data){};
  virtual void updateInteractiveData(const rm_referee::InteractiveData data, const ros::Time& last_get_){};
  virtual void updateEventDataData(const rm_msgs::EventData data){};
  virtual void updateEventDataData(const rm_msgs::EventData data, const ros::Time& last_get_){};
  virtual void updateJointStateData(const sensor_msgs::JointState::ConstPtr data){};
  virtual void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateActuatorStateData(const rm_msgs::ActuatorState::ConstPtr data){};
  virtual void updateActuatorStateData(const rm_msgs::ActuatorState::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateDbusData(const rm_msgs::DbusData::ConstPtr data){};
  virtual void updateDbusData(const rm_msgs::DbusData::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data){};
  virtual void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateVel2DCmdData(const geometry_msgs::Twist::ConstPtr data){};
  virtual void updateVel2DCmdData(const geometry_msgs::Twist::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr dat){};
  virtual void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateGimbalCmdData(const rm_msgs::GimbalCmd::ConstPtr data){};
  virtual void updateGimbalCmdData(const rm_msgs::GimbalCmd::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateCardCmdData(const rm_msgs::StateCmd::ConstPtr data){};
  virtual void updateCardCmdData(const rm_msgs::StateCmd::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateEngineerCmdData(const rm_msgs::EngineerCmd ::ConstPtr data){};
  virtual void updateEngineerCmdData(const rm_msgs::EngineerCmd ::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data){};
  virtual void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_){};
  virtual void updateRadarData(const std_msgs::Int8MultiArrayConstPtr data){};
  virtual void updateRadarData(const std_msgs::Int8MultiArrayConstPtr data, const ros::Time& last_get_){};
  virtual void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data){};
  virtual void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data, const ros::Time& last_get_){};

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
  explicit TriggerChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name);
  virtual void setContent(const std::string& content);
  virtual void display();
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
};
class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data) override;
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;
  void updateDbusData(const rm_msgs::DbusData::ConstPtr data) override;
  void updateCapacityData(const rm_msgs::CapacityData data) override;

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
  explicit ShooterTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data) override;
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
  explicit GimbalTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void updateGimbalCmdData(const rm_msgs::GimbalCmd ::ConstPtr data) override;
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
  explicit TargetTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data) override;
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

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
  explicit TimeChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name);
  virtual void display(const ros::Time& time);
  virtual void updateConfig(){};
};

class CapacitorTimeChangeUi : public TimeChangeUi
{
public:
  explicit CapacitorTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "capacitor"){};
  void add() override;
  void updateCapacityData(const rm_msgs::CapacityData data, const ros::Time& time) override;

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  double cap_power_;
};

class EffortTimeChangeUi : public TimeChangeUi
{
public:
  explicit EffortTimeChangeUi(ros::NodeHandle& nh, Base& base) : TimeChangeUi(nh, base, "effort"){};
  void updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time) override;

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
  void updateEngineerCmdData(const rm_msgs::EngineerCmd ::ConstPtr data, const ros::Time& last_get_) override;

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
  void updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data, const ros::Time& last_get_) override;

private:
  void display(const ros::Time& time) override;
  void updateConfig() override;
  uint8_t dart_launch_opening_status_;
};

//// FlashUi
class FlashUi : public UiBase
{
public:
  explicit FlashUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name);
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
  void updateRobotHurtData(const rm_msgs::RobotHurt data, const ros::Time& last_get_) override;

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
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_) override;

private:
  void display(const ros::Time& time) override;
  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(ros::NodeHandle& nh, Base& base) : FlashUi(nh, base, "spin"){};
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_) override;

private:
  void display(const ros::Time& time) override;
  uint8_t chassis_mode_;
};
}  // namespace rm_referee
