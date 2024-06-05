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
                   std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : UiBase(rpc_value, base, graph_queue, character_queue)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void display(const ros::Time& time){};
  virtual void updateConfig(){};
  void updateFlashUiForQueue(const ros::Time& time, bool state, bool once);
};

class CoverFlashUi : public FlashUi
{
public:
  explicit CoverFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                        std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "cover", graph_queue, character_queue){};
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data, const ros::Time& last_get_data_time) override;

private:
  void display(const ros::Time& time) override;

  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                       std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "spin", graph_queue, character_queue){};
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_data_time);

private:
  void display(const ros::Time& time) override;
  uint8_t chassis_mode_;
};

class HeroHitFlashUi : public FlashUi
{
public:
  explicit HeroHitFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                          std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, " hero_hit", graph_queue, character_queue)
  {
  }
  void updateHittingConfig(const rm_msgs::GameRobotHp& msg);

private:
  void display(const ros::Time& time) override;
  bool hitted_;
  rm_msgs::GameRobotHp last_hp_msg_;
};

class ExceedBulletSpeedFlashUi : public FlashUi
{
public:
  explicit ExceedBulletSpeedFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                    std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "exceed_bullet_speed", graph_queue, character_queue)
  {
  }
  void updateShootData(const rm_msgs::ShootData& msg);

private:
  void display(const ros::Time& time) override;
  rm_msgs::ShootData shoot_data_;
};

}  // namespace rm_referee
