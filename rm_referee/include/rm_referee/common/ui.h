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
  explicit UiBase(ros::NodeHandle& nh, DataTranslation& data_translation, const std::string& ui_type);
  virtual void add();

protected:
  DataTranslation& data_translation_;
  static int id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, Graph*> special_graph_vector_, normal_graph_vector_;
};

// TriggerChangeUi
class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(ros::NodeHandle& nh, DataTranslation& data_translation);
  void update(const std::string& graph_name, const std::string& content);
  virtual void update(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
                      bool sub_flag = false);
};

class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(ros::NodeHandle& nh, DataTranslation& data_translation);
  void update();
  void update(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
              bool sub_flag = false) override;

  void updateChassisMode(uint8_t mode);
  void updatePowerLimitState(uint8_t power_limit_state);
  void updateDbusData(uint8_t s_l, uint8_t s_r, uint8_t key_ctrl, uint8_t key_shift, uint8_t key_b);

private:
  static std::string getChassisState(uint8_t mode);

  Graph* chassis_graph_;
  uint8_t chassis_mode_, power_limit_state_, s_l_, s_r_, key_ctrl_, key_shift_, key_b_;
};
//
//// FixedUi
// class FixedUi : public UiBase
//{
// public:
//   explicit FixedUi(ros::NodeHandle& nh, DataTranslation& data_translation) : UiBase(nh, data_translation, "fixed"){};
//   void update();
//   void updateSpeedLimit(int speed_limit);
//
// private:
//   int getShootSpeedIndex();
//   int speed_limit_;
// };
//
//// FlashUi
// class FlashUi : public UiBase
//{
// public:
//   explicit FlashUi(ros::NodeHandle& nh, DataTranslation& data_translation) : UiBase(nh, data_translation, "flash"){};
//   void update(const std::string& name, const ros::Time& time, bool state = false);
//
// private:
//   void updateArmorPosition(const std::string& name, Graph* graph);
//   void updateChassisGimbalDate(const double yaw_joint, Graph* graph);
//   static uint8_t getArmorId(const std::string& name);
// };
//
//// TimeChangeUi
// class TimeChangeUi : public UiBase
//{
// public:
//   explicit TimeChangeUi(ros::NodeHandle& nh, DataTranslation& data_translation)
//     : UiBase(nh, data_translation, "time_change"){};
//   void add() override;
//   void update(const std::string& name, const ros::Time& time, double data = 0.);
// };
//
// class ShooterTriggerChangeUi : public TriggerChangeUi
//{
// public:
//  explicit ShooterTriggerChangeUi(ros::NodeHandle& nh);
//  void updateShooterMode(uint8_t mode);
//  void updateShooter(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
//                     bool sub_flag = false);
//
// private:
//  static std::string getShooterState(uint8_t mode);
//  uint8_t shooter_mode_;
//  Graph* shooter_graph_;
//};
//
// class GimbalTriggerChangeUi : public TriggerChangeUi
//{
// public:
//  explicit GimbalTriggerChangeUi(ros::NodeHandle& nh);
//  void updateGimbalMode(uint8_t mode);
//  void updateGimbal(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
//                    bool sub_flag = false);
//
// private:
//  static std::string getGimbalState(uint8_t mode);
//  uint8_t gimbal_mode_;
//  Graph* gimbal_graph_;
//};
//
// class TargetTriggerChangeUi : public TriggerChangeUi
//{
// public:
//  explicit TargetTriggerChangeUi(ros::NodeHandle& nh);
//  void updateTarget(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
//                    bool sub_flag = false);
//
// private:
//  std::string getTargetState(uint8_t target, uint8_t armor_target);
//  Graph* target_graph_;
//};
//
// class CapacitorTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit CapacitorTimeChangeUI(ros::NodeHandle& nh);
//  void add() override;
//  void updateCapPower(double cap_power);
//
// private:
//  void setCapacitorData();
//  double cap_power_;
//  Graph* capacitor_graph_;
//};
//
// class EffortTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit EffortTimeChangeUI(ros::NodeHandle& nh);
//  void updateEffort(std::string joint_name[], double joint_effort[]);
//
// private:
//  void setEffortData();
//  Graph* effort_graph_;
//  int size_t_ = 0;
//  std::string joint_name_[5];
//  double joint_effort_[5];
//};
//
// class ProgressTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit ProgressTimeChangeUI(ros::NodeHandle& nh);
//
// private:
//  void setProgressData(double data);
//  Graph* progress_graph_;
//};
//
// class TemperatureTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit TemperatureTimeChangeUI(ros::NodeHandle& nh);
//
// private:
//  void setTemperatureData();
//  Graph* temperature_graph_;
//};
//
// class OreRemindTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit OreRemindTimeChangeUI(ros::NodeHandle& nh);
//
// private:
//  void setOreRemindData();
//  Graph* ore_remind_graph_;
//};
//
// class DartStatusTimeChangeUI : public TimeChangeUi
//{
// public:
//  explicit DartStatusTimeChangeUI(ros::NodeHandle& nh);
//
// private:
//  void setDartStatusData();
//  Graph* dart_status_graph_;
//};
//
// class ArmorFlashUI : public FlashUi
//{
// public:
//  explicit ArmorFlashUI(ros::NodeHandle& nh, std::string armor_name);
//  void update(const std::string& name, const ros::Time& time, bool state);
//
// private:
//  uint8_t getArmorId(const std::string& name);
//  void updateArmorPosition(Graph* graph);
//  Graph* armor_graph_;
//  std::string armor_name_;
//};

}  // namespace rm_referee
