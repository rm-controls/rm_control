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
#ifndef RM_COMMON_REFEREE_H_
#define RM_COMMON_REFEREE_H_

#include <cstdint>
#include <ros/ros.h>

#include <rm_msgs/Referee.h>
#include <rm_msgs/SuperCapacitor.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include "rm_common/referee/data.h"

namespace rm_common
{
class SuperCapacitor
{
public:
  explicit SuperCapacitor(rm_common::CapacityData& data) : last_get_data_(ros::Time::now()), data_(data){};
  void read(const std::vector<uint8_t>& rx_buffer);
  ros::Time last_get_data_;

private:
  void dtpReceivedCallBack(unsigned char receive_byte);
  void receiveCallBack(unsigned char package_id, const unsigned char* data);
  static float int16ToFloat(unsigned short data0);
  rm_common::CapacityData& data_;
  unsigned char receive_buffer_[1024] = { 0 };
  unsigned char ping_pong_buffer_[1024] = { 0 };
  unsigned int receive_buf_counter_ = 0;
};

class Referee
{
public:
  Referee()
    : super_capacitor_(referee_data_.capacity_data)
    , last_get_(ros::Time::now())
    , last_send_(ros::Time::now())
    , client_id_(0)
  {
    referee_data_.robot_hurt_.hurt_type_ = 0x09;
  };
  void read();
  void addUi(const rm_common::GraphConfig& config, const std::string& content, bool priority_flag = false);
  void sendUi(const ros::Time& time);
  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void clearBuffer()
  {
    rx_buffer_.clear();
    for (int i = 0; i < 128; i++)
      tx_buffer_[i] = 0;
    rx_len_ = 0;
    tx_len_ = 0;
  }

  ros::Publisher referee_pub_;
  ros::Publisher super_capacitor_pub_;
  rm_common::RefereeData referee_data_{};
  std::vector<uint8_t> rx_buffer_;
  uint8_t tx_buffer_[128];
  int tx_len_, rx_len_;

private:
  int unpack(uint8_t* rx_data);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const;
  void getRobotInfo();
  void publishData();

  SuperCapacitor super_capacitor_;
  ros::Time last_get_, last_send_;
  rm_msgs::Referee referee_pub_data_;
  rm_msgs::SuperCapacitor super_capacitor_pub_data_;
  std::vector<std::pair<rm_common::GraphConfig, std::string>> ui_queue_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};
  int client_id_;
};

// CRC verification
uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8);
uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length);
uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc);
uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length);
void appendCRC16CheckSum(unsigned char* pch_message, unsigned int dw_length);
}  // namespace rm_common

#endif  // RM_COMMON_REFEREE_H_
