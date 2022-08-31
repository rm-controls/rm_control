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
#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "rm_referee/common/data.h"
#include "rm_referee/referee/referee_base.h"

namespace rm_referee
{
class SuperCapacitor
{
public:
  explicit SuperCapacitor(rm_referee::CapacityData& data) : last_get_data_(ros::Time::now()), data_(data){};
  void read(const std::vector<uint8_t>& rx_buffer);
  ros::Time last_get_data_;

private:
  void dtpReceivedCallBack(unsigned char receive_byte);
  void receiveCallBack(unsigned char package_id, const unsigned char* data);
  static float int16ToFloat(unsigned short data0);
  rm_referee::CapacityData& data_;
  unsigned char receive_buffer_[1024] = { 0 };
  unsigned char ping_pong_buffer_[1024] = { 0 };
  unsigned int receive_buf_counter_ = 0;
};

class Referee
{
public:
  Referee() : super_capacitor_(base_.capacity_data_ref_), last_get_(ros::Time::now())
  {
    base_.robot_hurt_data_.hurt_type = 0x09;
    // pub
    ros::NodeHandle root_nh;
    referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    game_robot_status_pub_ = root_nh.advertise<rm_msgs::GameRobotStatus>("/game_robot_status", 1);
    game_status_pub_ = root_nh.advertise<rm_msgs::GameStatus>("/game_status", 1);
    capacity_data_pub_ = root_nh.advertise<rm_msgs::CapacityData>("/capacity_data", 1);
    power_heat_data_pub_ = root_nh.advertise<rm_msgs::PowerHeatData>("/power_heat_data", 1);
    game_robot_hp_pub_ = root_nh.advertise<rm_msgs::GameRobotHp>("/game_robot_hp", 1);
    event_data_pub_ = root_nh.advertise<rm_msgs::EventData>("/event_data", 1);
    dart_status_pub_ = root_nh.advertise<rm_msgs::DartStatus>("/dart_status_data", 1);
    icra_buff_debuff_zone_status_pub_ =
        root_nh.advertise<rm_msgs::IcraBuffDebuffZoneStatus>("/icra_buff_debuff_zone_status_data", 1);
    supply_projectile_action_pub_ =
        root_nh.advertise<rm_msgs::SupplyProjectileAction>("/supply_projectile_action_data", 1);
    dart_remaining_time_pub_ = root_nh.advertise<rm_msgs::DartRemainingTime>("/dart_remaining_time_data", 1);
    robot_hurt_pub_ = root_nh.advertise<rm_msgs::RobotHurt>("/robot_hurt_data", 1);
    shoot_data_pub_ = root_nh.advertise<rm_msgs::ShootData>("/shoot_data", 1);
    bullet_remaining_pub_ = root_nh.advertise<rm_msgs::BulletRemaining>("/bullet_remaining_data", 1);
    rfid_status_pub_ = root_nh.advertise<rm_msgs::RfidStatus>("/rfid_status_data", 1);
    dart_client_cmd_pub_ = root_nh.advertise<rm_msgs::DartClientCmd>("/dart_client_cmd_data", 1);
    // initSerial
    base_.initSerial();
  };
  void read();
  void checkUiAdd()
  {
    if (referee_ui_->base_.dbus_data_.s_r == rm_msgs::DbusData::UP)
    {
      if (referee_ui_->add_ui_flag_)
      {
        referee_ui_->addUi();
        ROS_INFO("Add ui");
        referee_ui_->add_ui_flag_ = false;
      }
    }
    else
      referee_ui_->add_ui_flag_ = true;
  }
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher referee_pub_;
  ros::Publisher super_capacitor_pub_;
  ros::Publisher game_robot_status_pub_;
  ros::Publisher game_status_pub_;
  ros::Publisher capacity_data_pub_;
  ros::Publisher power_heat_data_pub_;
  ros::Publisher game_robot_hp_pub_;
  ros::Publisher event_data_pub_;
  ros::Publisher dart_status_pub_;
  ros::Publisher icra_buff_debuff_zone_status_pub_;
  ros::Publisher supply_projectile_action_pub_;
  ros::Publisher dart_remaining_time_pub_;
  ros::Publisher robot_hurt_pub_;
  ros::Publisher shoot_data_pub_;
  ros::Publisher bullet_remaining_pub_;
  ros::Publisher rfid_status_pub_;
  ros::Publisher dart_client_cmd_pub_;

  Base base_;
  std::vector<uint8_t> rx_buffer_;
  rm_referee::RefereeBase* referee_ui_;
  int rx_len_;

private:
  int unpack(uint8_t* rx_data);
  void getRobotInfo();
  void publishCapacityData();

  SuperCapacitor super_capacitor_;
  ros::Time last_get_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};
};

}  // namespace rm_referee
