//
// Created by gura on 24-5-29.
//

#include "rm_referee/ui/interactive_data.h"

namespace rm_referee
{
void InteractiveSender::sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data)
{
  uint8_t tx_data[sizeof(InteractiveData)] = { 0 };
  auto student_interactive_data = (InteractiveData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data.data_cmd_id = data_cmd_id;
  student_interactive_data->header_data.sender_id = base_.robot_id_;
  student_interactive_data->header_data.receiver_id = receiver_id;
  student_interactive_data->data = data;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(InteractiveData) + k_tail_length_);

  sendSerial(ros::Time::now(), sizeof(InteractiveData));
}

bool InteractiveSender::needSendInteractiveData()
{
  return ros::Time::now() - last_send_time_ > delay_;
}

void InteractiveSender::sendMapSentryData(const rm_referee::MapSentryData& data)
{
  uint8_t tx_data[sizeof(rm_referee::MapSentryData)] = { 0 };
  auto map_sentry_data = (rm_referee::MapSentryData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  map_sentry_data->intention = data.intention;
  map_sentry_data->start_position_x = data.start_position_x;
  map_sentry_data->start_position_y = data.start_position_y;
  for (int i = 0; i < 49; i++)
  {
    map_sentry_data->delta_x[i] = data.delta_x[i];
    map_sentry_data->delta_y[i] = data.delta_y[i];
  }
  map_sentry_data->sender_id = base_.robot_id_;
  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::MAP_SENTRY_CMD, sizeof(rm_referee::MapSentryData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_referee::MapSentryData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void CustomInfoSender::sendCustomInfoData(std::wstring data)
{
  if (data == last_custom_info_ || ros::Time::now() - last_send_ < ros::Duration(0.35))
    return;
  else
    last_custom_info_ = data;

  int data_len;
  rm_referee::CustomInfo tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::CustomInfo));

  tx_data.sender_id = base_.robot_id_;
  tx_data.receiver_id = base_.client_id_;

  uint16_t characters[15];
  for (int i = 0; i < 15; i++)
  {
    if (i < static_cast<int>(data.size()))
      characters[i] = static_cast<uint16_t>(data[i]);
    else
      characters[i] = static_cast<uint16_t>(L' ');
  }
  for (int i = 0; i < 15; i++)
  {
    tx_data.user_data[2 * i] = characters[i] & 0xFF;
    tx_data.user_data[2 * i + 1] = (characters[i] >> 8) & 0xFF;
  }
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::CUSTOM_INFO_CMD, data_len);
  last_send_ = ros::Time::now();
  sendSerial(ros::Time::now(), data_len);
}

void InteractiveSender::sendRadarInteractiveData(const rm_msgs::ClientMapReceiveData::ConstPtr& data)
{
  uint8_t tx_data[sizeof(rm_referee::ClientMapReceiveData)] = { 0 };
  auto radar_interactive_data = (rm_referee::ClientMapReceiveData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  radar_interactive_data->hero_position_x = data->hero_position_x;
  radar_interactive_data->hero_position_y = data->hero_position_y;
  radar_interactive_data->engineer_position_x = data->engineer_position_x;
  radar_interactive_data->engineer_position_y = data->engineer_position_y;
  radar_interactive_data->infantry_3_position_x = data->infantry_3_position_x;
  radar_interactive_data->infantry_3_position_y = data->infantry_3_position_y;
  radar_interactive_data->infantry_4_position_x = data->infantry_4_position_x;
  radar_interactive_data->infantry_4_position_y = data->infantry_4_position_y;
  radar_interactive_data->infantry_5_position_x = data->infantry_5_position_x;
  radar_interactive_data->infantry_5_position_y = data->infantry_5_position_y;
  radar_interactive_data->sentry_position_x = data->sentry_position_x;
  radar_interactive_data->sentry_position_y = data->sentry_position_y;
  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::CLIENT_MAP_CMD, sizeof(rm_referee::ClientMapReceiveData));
  tx_len_ =
      k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_referee::ClientMapReceiveData) + k_tail_length_);
  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void InteractiveSender::sendSentryCmdData(const rm_msgs::SentryCmdConstPtr& data)
{
  int data_len;
  rm_referee::SentryCmd tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::SentryCmd));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = REFEREE_SERVER;
  tx_data.sentry_info = data->sentry_info;
  tx_data.header.data_cmd_id = rm_referee::DataCmdId::SENTRY_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(ros::Time::now(), data_len);
}

void InteractiveSender::sendRadarCmdData(const rm_msgs::RadarInfoConstPtr& data)
{
  int data_len;
  rm_referee::RadarInfo tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::RadarInfo));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = REFEREE_SERVER;
  tx_data.radar_info = data->radar_info;

  tx_data.header.data_cmd_id = rm_referee::DataCmdId::RADAR_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(ros::Time::now(), data_len);
}

void BulletNumShare::sendBulletData()
{
  uint16_t receiver_id;
  if (base_.robot_color_ == "red")
    receiver_id = RED_HERO;
  else
    receiver_id = BLUE_HERO;
  if (count_receive_time_ % 5 == 1)
    count_receive_time_++;
  receiver_id += count_receive_time_ % 5;

  uint8_t tx_data[sizeof(BulletNumData)] = { 0 };
  auto bullet_num_data = (BulletNumData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  bullet_num_data->header_data.data_cmd_id = rm_referee::DataCmdId::BULLET_NUM_SHARE_CMD;
  bullet_num_data->header_data.sender_id = base_.robot_id_;
  bullet_num_data->header_data.receiver_id = receiver_id;
  bullet_num_data->bullet_42_mm_num = bullet_42_mm_num_;
  bullet_num_data->bullet_17_mm_num = bullet_17_mm_num_;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(BulletNumData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(BulletNumData) + k_tail_length_);
  sendSerial(ros::Time::now(), sizeof(BulletNumData));
  last_send_time_ = ros::Time::now();
  count_receive_time_++;
}

void BulletNumShare::updateBulletRemainData(const rm_msgs::BulletAllowance& data)
{
  if (data.bullet_allowance_num_42_mm > 5096 || data.bullet_allowance_num_17_mm > 5096)
    return;
  bullet_17_mm_num_ = data.bullet_allowance_num_17_mm;
  bullet_42_mm_num_ = data.bullet_allowance_num_42_mm;
}

void SentryToRadar::updateSentryAttackingTargetData(const rm_msgs::SentryAttackingTargetConstPtr& data)
{
  robot_id_ = data->target_robot_ID;
  target_position_x_ = data->target_position_x;
  target_position_y_ = data->target_position_y;
}

void SentryToRadar::sendSentryToRadarData()
{
  uint8_t tx_data[sizeof(SentryAttackingTargetData)] = { 0 };
  auto sentry_attacking_target_data = (SentryAttackingTargetData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  sentry_attacking_target_data->header_data.data_cmd_id = rm_referee::DataCmdId::SENTRY_TO_RADAR_CMD;
  sentry_attacking_target_data->header_data.sender_id = base_.robot_id_;
  if (base_.robot_color_ == "red")
    sentry_attacking_target_data->header_data.receiver_id = RED_RADAR;
  else
    sentry_attacking_target_data->header_data.receiver_id = BLUE_RADAR;
  sentry_attacking_target_data->target_robot_ID = robot_id_;
  sentry_attacking_target_data->target_position_x = target_position_x_;
  sentry_attacking_target_data->target_position_y = target_position_y_;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(SentryAttackingTargetData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(SentryAttackingTargetData) + k_tail_length_);
  sendSerial(ros::Time::now(), sizeof(SentryAttackingTargetData));
  last_send_time_ = ros::Time::now();
}

bool SentryToRadar::needSendInteractiveData()
{
  return InteractiveSender::needSendInteractiveData() && (robot_id_ != 0);
}

void RadarToSentry::updateRadarToSentryData(const rm_msgs::RadarToSentryConstPtr& data)
{
  robot_id_ = data->robot_ID;
  position_x_ = data->position_x;
  position_y_ = data->position_y;
  engineer_marked_ = data->engineer_marked;
  has_new_data_ = true;
}

void RadarToSentry::sendRadarToSentryData()
{
  uint8_t tx_data[sizeof(RadarToSentryData)] = { 0 };
  auto radar_to_sentry_data = (RadarToSentryData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  radar_to_sentry_data->header_data.data_cmd_id = rm_referee::DataCmdId::RADAR_TO_SENTRY_CMD;
  radar_to_sentry_data->header_data.sender_id = base_.robot_id_;
  if (base_.robot_color_ == "red")
    radar_to_sentry_data->header_data.receiver_id = RED_SENTRY;
  else
    radar_to_sentry_data->header_data.receiver_id = BLUE_SENTRY;
  radar_to_sentry_data->robot_ID = robot_id_;
  radar_to_sentry_data->position_x = position_x_;
  radar_to_sentry_data->position_y = position_y_;
  radar_to_sentry_data->engineer_marked = engineer_marked_;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(RadarToSentryData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(RadarToSentryData) + k_tail_length_);
  sendSerial(ros::Time::now(), sizeof(RadarToSentryData));
  last_send_time_ = ros::Time::now();
  has_new_data_ = false;
}

bool RadarToSentry::needSendInteractiveData()
{
  return (InteractiveSender::needSendInteractiveData() && has_new_data_);
}

}  // namespace rm_referee
