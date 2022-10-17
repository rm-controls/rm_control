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
class UiBase
{
public:
  explicit UiBase(ros::NodeHandle& nh, Base& base, const std::string& ui_type);
  virtual void add();

protected:
  Base& base_;
  static int id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, Graph*> graph_vector_;
};

class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(ros::NodeHandle& nh, Base& base);
  void update(const std::string& graph_name, const std::string& content);
  void update(const std::string& graph_name, uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0,
              bool sub_flag = false);

private:
  void updateConfig(const std::string& name, Graph* graph, uint8_t main_mode, bool main_flag, uint8_t sub_mode,
                    bool sub_flag);
  std::string getTargetState(uint8_t target, uint8_t armor_target);
  static std::string getChassisState(uint8_t mode);
  static std::string getGimbalState(uint8_t mode);
  static std::string getShooterState(uint8_t mode);
};

class TimeChangeUi : public UiBase
{
public:
  explicit TimeChangeUi(ros::NodeHandle& nh, Base& base) : UiBase(nh, base, "time_change"){};
  void add() override;
  void update(const std::string& name, const ros::Time& time, double data = 0.);

private:
  void setCapacitorData(Graph& graph);
  void setEffortData(Graph& graph);
  void setTemperatureData(Graph& graph);
  static void setProgressData(Graph& graph, double data);
  void setOreRemindData(Graph& graph);
  void setDartStatusData(Graph& graph);
};

class FixedUi : public UiBase
{
public:
  explicit FixedUi(ros::NodeHandle& nh, Base& base) : UiBase(nh, base, "fixed"){};
  void update();

private:
  int getShootSpeedIndex();
};

class FlashUi : public UiBase
{
public:
  explicit FlashUi(ros::NodeHandle& nh, Base& base) : UiBase(nh, base, "flash"){};
  void update(const std::string& name, const ros::Time& time, bool state = false);

private:
  void updateArmorPosition(const std::string& name, Graph* graph);
  void updateChassisGimbalDate(const double yaw_joint, Graph* graph);
  static uint8_t getArmorId(const std::string& name);
};

}  // namespace rm_referee
