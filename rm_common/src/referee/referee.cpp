/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by peter on 2021/5/17.
//
#include "rm_common/referee/referee.h"

namespace rm_common
{
// read data from referee
void Referee::read()
{
  uint8_t temp_buffer[256] = { 0 };
  int frame_len;
  if (ros::Time::now() - last_get_ > ros::Duration(0.1))
    referee_data_.is_online_ = false;
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int kI = 0; kI < k_unpack_buffer_length_ - rx_len_; ++kI)
      temp_buffer[kI] = unpack_buffer_[kI + rx_len_];
    for (int kI = 0; kI < rx_len_; ++kI)
      temp_buffer[kI + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[kI];
    for (int kI = 0; kI < k_unpack_buffer_length_; ++kI)
      unpack_buffer_[kI] = temp_buffer[kI];
  }
  for (int kI = 0; kI < k_unpack_buffer_length_ - k_frame_length_; ++kI)
  {
    if (unpack_buffer_[kI] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[kI]);
      if (frame_len != -1)
        kI += frame_len;
    }
  }
  super_capacitor_.read(rx_buffer_);
  getRobotInfo();
  publishData();
}

int Referee::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  rm_common::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (verifyCRC8CheckSum(rx_data, k_header_length_) == true)
  {
    frame_len = frame_header.data_length_ + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (verifyCRC16CheckSum(rx_data, frame_len) == true)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case rm_common::RefereeCmdId::GAME_STATUS_CMD:
        {
          memcpy(&referee_data_.game_status_, rx_data + 7, sizeof(rm_common::GameStatus));
          break;
        }
        case rm_common::RefereeCmdId::GAME_RESULT_CMD:
        {
          memcpy(&referee_data_.game_result_, rx_data + 7, sizeof(rm_common::GameResult));
          break;
        }
        case rm_common::RefereeCmdId::GAME_ROBOT_HP_CMD:
        {
          memcpy(&referee_data_.game_robot_hp_, rx_data + 7, sizeof(rm_common::GameRobotHp));
          break;
        }
        case rm_common::RefereeCmdId::DART_STATUS_CMD:
        {
          memcpy(&referee_data_.dart_status_, rx_data + 7, sizeof(rm_common::DartStatus));
          break;
        }
        case rm_common::RefereeCmdId::ICRA_ZONE_STATUS_CMD:
        {
          memcpy(&referee_data_.icra_buff_debuff_zone_status, rx_data + 7, sizeof(rm_common::IcraBuffDebuffZoneStatus));
          break;
        }
        case rm_common::RefereeCmdId::FIELD_EVENTS_CMD:
        {
          memcpy(&referee_data_.event_data_, rx_data + 7, sizeof(rm_common::EventData));
          break;
        }
        case rm_common::RefereeCmdId::SUPPLY_PROJECTILE_ACTION_CMD:
        {
          memcpy(&referee_data_.supply_projectile_action_, rx_data + 7, sizeof(rm_common::SupplyProjectileAction));
          break;
        }
        case rm_common::RefereeCmdId::REFEREE_WARNING_CMD:
        {
          memcpy(&referee_data_.referee_warning_, rx_data + 7, sizeof(rm_common::RefereeWarning));
          break;
        }
        case rm_common::RefereeCmdId::DART_REMAINING_CMD:
        {
          memcpy(&referee_data_.dart_remaining_time_, rx_data + 7, sizeof(rm_common::DartRemainingTime));
          break;
        }
        case rm_common::RefereeCmdId::ROBOT_STATUS_CMD:
        {
          memcpy(&referee_data_.game_robot_status_, rx_data + 7, sizeof(rm_common::GameRobotStatus));
          break;
        }
        case rm_common::RefereeCmdId::POWER_HEAT_DATA_CMD:
        {
          memcpy(&referee_data_.power_heat_data_, rx_data + 7, sizeof(rm_common::PowerHeatData));
          referee_data_.power_heat_data_.chassis_volt_ =
              (uint16_t)(referee_data_.power_heat_data_.chassis_volt_ * 0.001);  // mV->V
          referee_data_.power_heat_data_.chassis_current_ =
              (uint16_t)(referee_data_.power_heat_data_.chassis_current_ * 0.001);  // mA->A
          break;
        }
        case rm_common::RefereeCmdId::ROBOT_POS_CMD:
        {
          memcpy(&referee_data_.game_robot_pos_, rx_data + 7, sizeof(rm_common::GameRobotPos));
          break;
        }
        case rm_common::RefereeCmdId::BUFF_CMD:
        {
          memcpy(&referee_data_.buff_, rx_data + 7, sizeof(rm_common::Buff));
          break;
        }
        case rm_common::RefereeCmdId::AERIAL_ROBOT_ENERGY_CMD:
        {
          memcpy(&referee_data_.aerial_robot_energy_, rx_data + 7, sizeof(rm_common::AerialRobotEnergy));
          break;
        }
        case rm_common::RefereeCmdId::ROBOT_HURT_CMD:
        {
          memcpy(&referee_data_.robot_hurt_, rx_data + 7, sizeof(rm_common::RobotHurt));
          break;
        }
        case rm_common::RefereeCmdId::SHOOT_DATA_CMD:
        {
          memcpy(&referee_data_.shoot_data_, rx_data + 7, sizeof(rm_common::ShootData));
          break;
        }
        case rm_common::RefereeCmdId::BULLET_REMAINING_CMD:
        {
          memcpy(&referee_data_.bullet_remaining_, rx_data + 7, sizeof(rm_common::BulletRemaining));
          break;
        }
        case rm_common::RefereeCmdId::ROBOT_RFID_STATUS_CMD:
        {
          memcpy(&referee_data_.rfid_status_, rx_data + 7, sizeof(rm_common::RfidStatus));
          break;
        }
        case rm_common::RefereeCmdId::DART_CLIENT_CMD:
        {
          memcpy(&referee_data_.dart_client_cmd_, rx_data + 7, sizeof(rm_common::DartClientCmd));
          break;
        }
        case rm_common::RefereeCmdId::INTERACTIVE_DATA_CMD:
        {
          memcpy(&referee_data_.interactive_data, rx_data + 7, sizeof(rm_common::InteractiveData));
          break;
        }
        default:
          ROS_WARN("Referee command ID not found.");
          break;
      }
      referee_data_.is_online_ = true;
      last_get_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

void Referee::getRobotInfo()
{
  referee_data_.robot_id_ = referee_data_.game_robot_status_.robot_id_;
  referee_data_.robot_color_ = referee_data_.robot_id_ >= 100 ? "blue" : "red";
  if (referee_data_.robot_id_ != rm_common::RobotId::BLUE_SENTRY &&
      referee_data_.robot_id_ != rm_common::RobotId::RED_SENTRY)
  {
    switch (referee_data_.robot_id_)
    {
      case rm_common::RobotId::BLUE_HERO:
        client_id_ = rm_common::ClientId::BLUE_HERO_CLIENT;
        break;
      case rm_common::RobotId::BLUE_ENGINEER:
        client_id_ = rm_common::ClientId::BLUE_ENGINEER_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_3:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_3_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_4:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_4_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_5:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_5_CLIENT;
        break;
      case rm_common::RobotId::RED_HERO:
        client_id_ = rm_common::ClientId::RED_HERO_CLIENT;
        break;
      case rm_common::RobotId::RED_ENGINEER:
        client_id_ = rm_common::ClientId::RED_ENGINEER_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_3:
        client_id_ = rm_common::ClientId::RED_STANDARD_3_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_4:
        client_id_ = rm_common::ClientId::RED_STANDARD_4_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_5:
        client_id_ = rm_common::ClientId::RED_STANDARD_5_CLIENT;
        break;
    }
  }
}

void Referee::publishData()
{
  if (referee_data_.robot_id_ == rm_common::RobotId::RED_HERO ||
      referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO)
  {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_;
  }
  else
  {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_;
  }
  referee_pub_data_.chassis_volt = referee_data_.power_heat_data_.chassis_volt_;
  referee_pub_data_.chassis_current = referee_data_.power_heat_data_.chassis_current_;
  referee_pub_data_.chassis_power = referee_data_.power_heat_data_.chassis_power_;
  referee_pub_data_.chassis_power_buffer = referee_data_.power_heat_data_.chassis_power_buffer_;
  referee_pub_data_.robot_hp = referee_data_.game_robot_status_.remain_hp_;
  referee_pub_data_.hurt_armor_id = referee_data_.robot_hurt_.armor_id_;
  referee_pub_data_.hurt_type = referee_data_.robot_hurt_.hurt_type_;
  referee_pub_data_.bullet_speed = referee_data_.shoot_data_.bullet_speed_;
  referee_pub_data_.stamp = last_get_;

  super_capacitor_pub_data_.capacity = (float)referee_data_.capacity_data.cap_power_;
  super_capacitor_pub_data_.chassis_power_buffer = (uint16_t)referee_data_.capacity_data.buffer_power_;
  super_capacitor_pub_data_.limit_power = (float)referee_data_.capacity_data.limit_power_;
  super_capacitor_pub_data_.chassis_power = (float)referee_data_.capacity_data.chassis_power_;
  super_capacitor_pub_data_.stamp = super_capacitor_.last_get_data_;

  referee_pub_.publish(referee_pub_data_);
  super_capacitor_pub_.publish(super_capacitor_pub_data_);
}

void Referee::sendInteractiveData(int data_cmd_id, int receiver_id, uint8_t data)
{
  uint8_t tx_data[sizeof(rm_common::InteractiveData)] = { 0 };
  auto student_interactive_data = (rm_common::InteractiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data_.data_cmd_id_ = data_cmd_id;
  student_interactive_data->header_data_.sender_id_ = referee_data_.robot_id_;
  student_interactive_data->header_data_.receiver_id_ = receiver_id;
  student_interactive_data->data_ = data;
  pack(tx_buffer_, tx_data, rm_common::RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(rm_common::InteractiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + (int)sizeof(rm_common::InteractiveData) + k_tail_length_;
}

void Referee::addUi(const rm_common::GraphConfig& config, const std::string& content, bool priority_flag)
{
  for (int i = 0; i < (int)ui_queue_.size() - 20; i++)
    ui_queue_.erase(ui_queue_.begin());
  if (priority_flag)
    ui_queue_.push_back(std::pair<rm_common::GraphConfig, std::string>(config, content));
  else
    ui_queue_.insert(ui_queue_.begin(), std::pair<rm_common::GraphConfig, std::string>(config, content));
}

void Referee::sendUi(const ros::Time& time)
{
  if (ui_queue_.empty() || time - last_send_ < ros::Duration(0.05))
    return;
  rm_common::GraphData tx_data;
  int data_len = (int)sizeof(rm_common::GraphData);
  tx_data.header_.sender_id_ = referee_data_.robot_id_;
  tx_data.header_.receiver_id_ = client_id_;
  tx_data.config_ = ui_queue_.back().first;
  if (ui_queue_.back().second.empty())
  {
    tx_data.header_.data_cmd_id_ = rm_common::DataCmdId::CLIENT_GRAPH_SINGLE_CMD;
    data_len -= 30;
  }
  else
  {
    tx_data.header_.data_cmd_id_ = rm_common::DataCmdId::CLIENT_CHARACTER_CMD;
    for (int i = 0; i < 30; i++)
    {
      if (i < (int)ui_queue_.back().second.size())
        tx_data.content_[i] = ui_queue_.back().second[i];
      else
        tx_data.content_[i] = ' ';
    }
  }
  pack(tx_buffer_, (uint8_t*)&tx_data, rm_common::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  tx_len_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_ + data_len;
  ui_queue_.pop_back();
  last_send_ = time;
}

void Referee::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = (rm_common::FrameHeader*)tx_buffer;

  frame_header->sof_ = 0xA5;
  frame_header->data_length_ = len;
  memcpy(&tx_buffer[k_header_length_], (uint8_t*)&cmd_id, k_cmd_id_length_);
  appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, len);
  appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + len + k_tail_length_);
}

uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8)
{
  unsigned char uc_index;
  while (dw_length--)
  {
    uc_index = uc_crc_8 ^ (*pch_message++);
    uc_crc_8 = rm_common::kCrc8Table[uc_index];
  }
  return (uc_crc_8);
}

uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
{
  unsigned char uc_expected;
  if ((pch_message == nullptr) || (dw_length <= 2))
    return 0;
  uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, rm_common::kCrc8Init);
  return (uc_expected == pch_message[dw_length - 1]);
}

void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
{
  unsigned char uc_crc;
  if ((pch_message == nullptr) || (dw_length <= 2))
    return;
  uc_crc = getCRC8CheckSum((unsigned char*)pch_message, dw_length - 1, rm_common::kCrc8Init);
  pch_message[dw_length - 1] = uc_crc;
}

uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc)
{
  uint8_t chData;
  if (pch_message == nullptr)
    return 0xFFFF;
  while (dw_length--)
  {
    chData = *pch_message++;
    (w_crc) = ((uint16_t)(w_crc) >> 8) ^ rm_common::wCRC_table[((uint16_t)(w_crc) ^ (uint16_t)(chData)) & 0x00ff];
  }
  return w_crc;
}

uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
{
  uint16_t w_expected;
  if ((pch_message == nullptr) || (dw_length <= 2))
    return 0;
  w_expected = getCRC16CheckSum(pch_message, dw_length - 2, rm_common::kCrc16Init);
  return ((w_expected & 0xff) == pch_message[dw_length - 2] && ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
}

void appendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
{
  uint16_t wCRC;
  if ((pch_message == nullptr) || (dw_length <= 2))
    return;
  wCRC = getCRC16CheckSum((uint8_t*)pch_message, dw_length - 2, rm_common::kCrc16Init);
  pch_message[dw_length - 2] = (uint8_t)(wCRC & 0x00ff);
  pch_message[dw_length - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

void SuperCapacitor::read(const std::vector<uint8_t>& rx_buffer)
{
  int count = 0;
  memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
  memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
  receive_buf_counter_ = 0;
  for (unsigned char kI : rx_buffer)
  {
    dtpReceivedCallBack(kI);
    count++;
    if (count >= (int)sizeof(receive_buffer_))
    {
      memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
      memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
      receive_buf_counter_ = 0;
    }
  }
  if (data_.chassis_power_ >= 120.)
    data_.chassis_power_ = 120.;
  if (data_.chassis_power_ <= 0.)
    data_.chassis_power_ = 0.;
  if (data_.buffer_power_ >= 25.)
    data_.buffer_power_ = 25.;
  if (data_.buffer_power_ <= 0.)
    data_.buffer_power_ = 0.;
  if (data_.cap_power_ >= 1.)
    data_.cap_power_ = 1.;
  if (ros::Time::now() - last_get_data_ > ros::Duration(0.1))
    data_.is_online_ = false;
}

void SuperCapacitor::receiveCallBack(unsigned char package_id, const unsigned char* data)
{
  if (package_id == 0)
  {
    last_get_data_ = ros::Time::now();
    data_.is_online_ = true;
    data_.chassis_power_ = (double)int16ToFloat((data[0] << 8) | data[1]);
    data_.limit_power_ = (double)int16ToFloat((data[2] << 8) | data[3]);
    data_.buffer_power_ = (double)int16ToFloat((data[4] << 8) | data[5]);
    data_.cap_power_ = (double)int16ToFloat((data[6] << 8) | data[7]);
  }
}

void SuperCapacitor::dtpReceivedCallBack(unsigned char receive_byte)
{
  unsigned char check_flag;
  unsigned int sof_pos, eof_pos, check_counter;

  receive_buffer_[receive_buf_counter_] = receive_byte;
  receive_buf_counter_ = receive_buf_counter_ + 1;
  check_flag = 0;
  sof_pos = 0;
  eof_pos = 0;
  check_counter = 0;
  while (true)
  {
    if (check_flag == 0 && receive_buffer_[check_counter] == 0xff)
    {
      check_flag = 1;
      sof_pos = check_counter;
    }
    else if (check_flag == 1 && receive_buffer_[check_counter] == 0xff)
    {
      eof_pos = check_counter;
      break;
    }
    if (check_counter >= (receive_buf_counter_ - 1))
      break;
    else
      check_counter++;
  }  // Find Package In Buffer

  if ((eof_pos - sof_pos) == 11)
  {
    unsigned int temp_var;
    unsigned char data_buffer[8] = { 0 };
    unsigned char valid_buffer[12] = { 0 };

    for (temp_var = 0; temp_var < 12; temp_var++)  // Copy Data To Another Buffer
      valid_buffer[temp_var] = receive_buffer_[sof_pos + temp_var];

    eof_pos++;
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_ - eof_pos; temp_var++)
      ping_pong_buffer_[temp_var] = receive_buffer_[eof_pos + temp_var];
    receive_buf_counter_ = receive_buf_counter_ - eof_pos;
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_; temp_var++)
      receive_buffer_[temp_var] = ping_pong_buffer_[temp_var];

    unsigned char pid_bit = valid_buffer[1] >> 4;  // Get The PID Bit
    if (pid_bit == ((~(valid_buffer[1] & 0x0f)) & 0x0f))
    {  // PID Verify
      for (temp_var = 0; temp_var < 8; ++temp_var)
        data_buffer[temp_var] = valid_buffer[2 + temp_var];
      if (valid_buffer[10] != 0x00)
      {  // Some Byte had been replace
        unsigned char temp_filter = 0x00;
        for (temp_var = 0; temp_var < 8; ++temp_var)
          if (((valid_buffer[10] & (temp_filter | (0x01 << temp_var))) >> temp_var) == 1)  // This Byte Need To Adjust
            data_buffer[temp_var] = 0xff;                                                  // Adjust to 0xff
      }
      receiveCallBack(pid_bit, data_buffer);
    }
  }
  else if ((eof_pos - sof_pos) != 0 && eof_pos != 0)
  {
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    receive_buf_counter_ = 0;
  }
}

float SuperCapacitor::int16ToFloat(unsigned short data0)
{
  if (data0 == 0)
    return 0;
  float* fp32;
  unsigned int fInt32 =
      ((data0 & 0x8000) << 16) | (((((data0 >> 10) & 0x1f) - 0x0f + 0x7f) & 0xff) << 23) | ((data0 & 0x03FF) << 13);
  fp32 = (float*)&fInt32;
  return *fp32;
}
}  // namespace rm_common
