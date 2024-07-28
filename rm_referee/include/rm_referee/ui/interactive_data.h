//
// Created by gura on 24-5-29.
//

#pragma once

#include "rm_referee/ui/ui_base.h"

namespace rm_referee
{
class InteractiveSender : public UiBase
{
public:
  explicit InteractiveSender(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                             std::deque<Graph>* character_queue = nullptr)
    : UiBase(rpc_value, base, graph_queue, character_queue){};

  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void sendRadarInteractiveData(const rm_msgs::ClientMapReceiveData::ConstPtr& data);
  void sendMapSentryData(const rm_referee::MapSentryData& data);
  void sendSentryCmdData(const rm_msgs::SentryCmdConstPtr& data);
  void sendRadarCmdData(const rm_msgs::RadarInfoConstPtr& data);
  virtual bool needSendInteractiveData();
  ros::Time last_send_time_;
};

class CustomInfoSender : public InteractiveSender
{
public:
  explicit CustomInfoSender(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                            std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void sendCustomInfoData(std::wstring data);

protected:
  std::wstring last_custom_info_;
};

class BulletNumShare : public InteractiveSender
{
public:
  explicit BulletNumShare(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                          std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void sendBulletData();
  void updateBulletRemainData(const rm_msgs::BulletAllowance& data);
  int bullet_42_mm_num_, bullet_17_mm_num_, count_receive_time_{ 0 };
};

class SentryToRadar : public InteractiveSender
{
public:
  explicit SentryToRadar(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                         std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void updateSentryAttackingTargetData(const rm_msgs::SentryAttackingTargetConstPtr& data);
  bool needSendInteractiveData() override;
  void sendSentryToRadarData();
  int robot_id_;
  float target_position_x_, target_position_y_;
};

class RadarToSentry : public InteractiveSender
{
public:
  explicit RadarToSentry(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                         std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void updateRadarToSentryData(const rm_msgs::RadarToSentryConstPtr& data);
  bool needSendInteractiveData() override;
  void sendRadarToSentryData();
  int robot_id_;
  float position_x_, position_y_;
  bool engineer_marked_{ false }, has_new_data_{ false };
};

}  // namespace rm_referee
