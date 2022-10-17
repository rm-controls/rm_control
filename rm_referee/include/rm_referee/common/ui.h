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
  virtual void update();
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false);
};

class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void ChassisModeCallBack(uint8_t mode);
  void PowerLimitStateCallBack(uint8_t power_limit_state);
  void DbusDataCallBack(uint8_t s_l, uint8_t s_r, uint8_t key_ctrl, uint8_t key_shift, uint8_t key_b);
  void capacityDataCallBack();

private:
  void update() override;
  void updateCapacity();
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  static std::string getChassisState(uint8_t mode);

  uint8_t chassis_mode_, power_limit_state_, s_l_, s_r_, key_ctrl_, key_shift_, key_b_;
};

class ShooterTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ShooterTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void ShooterModeCallBack(uint8_t mode);
  void ShootFrequencyCallBack(uint8_t shoot_frequency);

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  static std::string getShooterState(uint8_t mode);

  uint8_t shooter_mode_, shoot_frequency_;
};

class GimbalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit GimbalTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void GimbalModeCallback(uint8_t mode);
  void GimbalEjectCallBack(uint8_t gimbal_eject);

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  static std::string getGimbalState(uint8_t mode);

  uint8_t gimbal_mode_, gimbal_eject_;
};

class TargetTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetTriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void ManalToRefereeCallback(uint8_t det_target, uint8_t shoot_frequency, uint8_t det_armor_target, uint8_t det_color,
                              uint8_t gimbal_eject);
  void ShooterCallBack();

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getTargetState(uint8_t target, uint8_t armor_target);

  uint8_t det_target_, shoot_frequency_, det_armor_target_, det_color_, gimbal_eject_;
};

//// FixedUi
class FixedUi : public UiBase
{
public:
  explicit FixedUi(ros::NodeHandle& nh, Base& base) : UiBase(nh, base, "fixed"){};
  void update();
  void speedLimitCallback(int speed_limit);

private:
  int getShootSpeedIndex();
  int speed_limit_;
};

//// TimeChangeUi
class TimeChangeUi : public UiBase
{
public:
  explicit TimeChangeUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name);
  void add() override;
  virtual void update(const ros::Time& time);

private:
  virtual void updateData();
};

class CapacitorTimeChangeUI : public TimeChangeUi
{
public:
  explicit CapacitorTimeChangeUI(ros::NodeHandle& nh, Base& base);
  void add() override;

  void capPowerDataCallBack(double cap_power, ros::Time& time);

private:
  void updateData() override;
  double cap_power_;
};

class EffortTimeChangeUI : public TimeChangeUi
{
public:
  explicit EffortTimeChangeUI(ros::NodeHandle& nh, Base& base);

  void EffortDataCallBack(std::string joint_name, double joint_effort);

private:
  void updateData() override;
  double joint_effort_;
  std::string joint_name_;
};

class ProgressTimeChangeUI : public TimeChangeUi
{
public:
  explicit ProgressTimeChangeUI(ros::NodeHandle& nh, Base& base);

  void progressDataCallBack(uint32_t finished_data, uint32_t total_steps_, std::string step_name_);

private:
  void updateData() override;
  uint32_t finished_data_, total_steps_;
  std::string step_name_;
};

class DartStatusTimeChangeUI : public TimeChangeUi
{
public:
  explicit DartStatusTimeChangeUI(ros::NodeHandle& nh, Base& base);

  void dartLaunchOpeningStatusCallBack(uint8_t dart_launch_opening_status);

private:
  void updateData() override;
  uint8_t dart_launch_opening_status_;
};

class OreRemindTimeChangeUI : public TimeChangeUi
{
public:
  explicit OreRemindTimeChangeUI(ros::NodeHandle& nh, Base& base);

  void stageRemainTimeCallBack(uint16_t stage_remain_time);

private:
  void updateData() override;
  uint16_t stage_remain_time_;
};

//// FlashUi
class FlashUi : public UiBase
{
public:
  explicit FlashUi(ros::NodeHandle& nh, Base& base, const std::string& graph_name);
  virtual void update(const ros::Time& time, bool state = false);

private:
  virtual void updateData();
};

class AuxFlashUI : public FlashUi
{
public:
  explicit AuxFlashUI(ros::NodeHandle& nh, Base& base_);

  void jointStateCallBack(uint32_t joint_position);

private:
  void updateData() override;
  uint32_t joint_position_;
};

class CoverFlashUI : public FlashUi
{
public:
  explicit CoverFlashUI(ros::NodeHandle& nh, Base& base_);
  void coverStateCallBack(uint8_t cover_state);
  virtual void update(const ros::Time& time, bool state = false) override;

private:
  uint8_t cover_state_;
};

}  // namespace rm_referee
