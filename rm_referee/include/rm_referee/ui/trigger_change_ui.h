//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include "rm_referee/ui/ui_base.h"
#include <rm_common/decision/power_limit.h>
#include "std_msgs/String.h"

namespace rm_referee
{
class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                           std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : UiBase(rpc_value, base, graph_queue, character_queue)
  {
    if (graph_name == "chassis")
      graph_ = new Graph(rpc_value["config"], base_, 1);
    else
      graph_ = new Graph(rpc_value["config"], base_, id_++);
  };
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
  void updateForQueue() override;
  void updateForQueue(bool check_repeat);
  void updateTwiceForQueue(bool check_repeat = true);
};

class TriggerChangeGroupUi : public GroupUiBase
{
public:
  explicit TriggerChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                                std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : GroupUiBase(rpc_value, base, graph_queue, character_queue)
  {
    graph_name_ = graph_name;
  };
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
  void updateForQueue() override;
  void updateForQueue(bool check_repeat);
  void updateTwiceForQueue(bool check_repeat = true);

protected:
  std::string graph_name_;
};

class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                  std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "chassis", graph_queue, character_queue)
  {
    if (base.robot_id_ == rm_referee::RobotId::RED_ENGINEER || base.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
      graph_->setContent("raw");
    else
      graph_->setContent("follow");

    if (rpc_value.hasMember("mode_change_threshold"))
      mode_change_threshold_ = static_cast<double>(rpc_value["mode_change_threshold"]);
    else
      mode_change_threshold_ = 0.7;
  }
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr& data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;
  void updateDbusData(const rm_msgs::DbusData::ConstPtr& data);
  void updateCapacityResetStatus();
  void checkModeChange();

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  void displayInCapacity();
  std::string getChassisState(uint8_t mode);
  uint8_t chassis_mode_, power_limit_state_, s_l_, s_r_, key_ctrl_, key_shift_, key_b_;
  double mode_change_threshold_;
};

class ShooterTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ShooterTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                  std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "shooter", graph_queue, character_queue)
  {
    graph_->setContent("0");
  }
  void updateShootStateData(const rm_msgs::ShootState::ConstPtr& data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getShooterState(uint8_t mode);
  uint8_t shooter_mode_, shoot_frequency_;
};

class GimbalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit GimbalTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                 std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "gimbal", graph_queue, character_queue)
  {
    graph_->setContent("0");
  }
  void updateGimbalCmdData(const rm_msgs::GimbalCmd ::ConstPtr& data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getGimbalState(uint8_t mode);
  uint8_t gimbal_mode_, gimbal_eject_;
};

class TargetTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                 std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "target", graph_queue, character_queue)
  {
    graph_->setContent("armor");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateShootStateData(const rm_msgs::ShootState::ConstPtr& data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getTargetState(uint8_t target, uint8_t armor_target);
  uint8_t det_target_, shoot_frequency_, det_armor_target_, det_color_, gimbal_eject_;
};

class TargetViewAngleTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetViewAngleTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                          std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "target_scale", graph_queue, character_queue)
  {
  }
  void updateTrackID(int id);

private:
  void update() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  int track_id_;
};

class PolygonTriggerChangeGroupUi : public TriggerChangeGroupUi
{
public:
  explicit PolygonTriggerChangeGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                       std::deque<Graph>* character_queue)
    : TriggerChangeGroupUi(rpc_value, base, "Polygon", graph_queue, character_queue)
  {
    ROS_ASSERT(rpc_value.hasMember("points"));
    XmlRpc::XmlRpcValue config;

    config["type"] = "line";

    if (rpc_value["graph_config"].hasMember("color"))
      config["color"] = rpc_value["graph_config"]["color"];
    else
      config["color"] = "cyan";
    if (rpc_value["graph_config"].hasMember("width"))
      config["width"] = rpc_value["graph_config"]["width"];
    else
      config["width"] = 2;
    XmlRpc::XmlRpcValue points = rpc_value["points"];
    config["start_position"].setSize(2);
    config["end_position"].setSize(2);
    for (int i = 1; i <= points.size(); i++)
    {
      if (i != points.size())
      {
        config["start_position"][0] = points[i - 1][0];
        config["start_position"][1] = points[i - 1][1];
        config["end_position"][0] = points[i][0];
        config["end_position"][1] = points[i][1];
      }
      else
      {
        // Connect first and last point
        config["start_position"][0] = points[i - 1][0];
        config["start_position"][1] = points[i - 1][1];
        config["end_position"][0] = points[0][0];
        config["end_position"][1] = points[0][1];
      }
      graph_vector_.insert(
          std::make_pair<std::string, Graph*>("graph_" + std::to_string(i), new Graph(config, base_, id_++)));
    }
  }
  void update() override;
};

class CameraTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit CameraTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                 std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, "camera", graph_queue, character_queue)
  {
    if (rpc_value.hasMember("camera_name"))
    {
      XmlRpc::XmlRpcValue& data = rpc_value["camera_name"];
      camera1_name_ = static_cast<std::string>(data["camera1_name"]);
      camera2_name_ = static_cast<std::string>(data["camera2_name"]);
    }
    else
      ROS_WARN("Camera config 's member 'camera_name' not defined.");
    graph_->setContent("0");
  }
  void updateCameraName(const std_msgs::StringConstPtr& data);

private:
  void update() override;
  void updateConfig(uint8_t main_mode = 0, bool main_flag = false, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string current_camera_{}, camera1_name_{}, camera2_name_{};
};

class StringTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit StringTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& name,
                                 std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : TriggerChangeUi(rpc_value, base, name, graph_queue, character_queue){};
  void updateStringUiData(const std::string& data);

private:
  void update() override;
  std::string data_;
};

}  // namespace rm_referee
