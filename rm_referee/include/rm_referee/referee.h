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
#include "rm_referee/referee_base.h"

namespace rm_referee
{
class Referee
{
public:
  Referee(ros::NodeHandle& nh) : referee_ui_(nh, base_), last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("New serial protocol loading.");
    // pub
    game_robot_status_pub_ = nh.advertise<rm_msgs::GameRobotStatus>("game_robot_status", 1);
    game_status_pub_ = nh.advertise<rm_msgs::GameStatus>("game_status", 1);
    power_heat_data_pub_ = nh.advertise<rm_msgs::PowerHeatData>("power_heat_data", 1);
    game_robot_hp_pub_ = nh.advertise<rm_msgs::GameRobotHp>("game_robot_hp", 1);
    current_sentry_pos_pub_ = nh.advertise<rm_msgs::CurrentSentryPosData>("current_sentry_pos", 1);
    buff_pub_ = nh.advertise<rm_msgs::Buff>("robot_buff", 1);
    event_data_pub_ = nh.advertise<rm_msgs::EventData>("event_data", 1);
    dart_status_pub_ = nh.advertise<rm_msgs::DartStatus>("dart_status_data", 1);
    icra_buff_debuff_zone_status_pub_ =
        nh.advertise<rm_msgs::IcraBuffDebuffZoneStatus>("icra_buff_debuff_zone_status_data", 1);
    supply_projectile_action_pub_ = nh.advertise<rm_msgs::SupplyProjectileAction>("supply_projectile_action_data", 1);
    dart_remaining_time_pub_ = nh.advertise<rm_msgs::DartRemainingTime>("dart_remaining_time_data", 1);
    robot_hurt_pub_ = nh.advertise<rm_msgs::RobotHurt>("robot_hurt_data", 1);
    shoot_data_pub_ = nh.advertise<rm_msgs::ShootData>("shoot_data", 1);
    bullet_allowance_pub_ = nh.advertise<rm_msgs::BulletAllowance>("bullet_allowance_data", 1);
    rfid_status_pub_ = nh.advertise<rm_msgs::RfidStatus>("rfid_status_data", 1);
    dart_client_cmd_pub_ = nh.advertise<rm_msgs::DartClientCmd>("dart_client_cmd_data", 1);
    client_map_receive_pub_ = nh.advertise<rm_msgs::ClientMapReceiveData>("client_map_receive", 1);
    robots_position_pub_ = nh.advertise<rm_msgs::RobotsPositionData>("robot_position", 1);
    radar_mark_pub_ = nh.advertise<rm_msgs::RadarMarkData>("radar_mark", 1);
    client_map_send_data_pub_ = nh.advertise<rm_msgs::ClientMapSendData>("client_map_send_data", 1);
    game_robot_pos_pub_ = nh.advertise<rm_msgs::GameRobotPosData>("game_robot_pos", 1);
    sentry_info_pub_ = nh.advertise<rm_msgs::SentryInfo>("sentry_info", 1);
    radar_info_pub_ = nh.advertise<rm_msgs::RadarInfo>("radar_info", 1);

    ros::NodeHandle power_management_nh = ros::NodeHandle(nh, "power_management");
    power_management_sample_and_status_data_pub_ =
        power_management_nh.advertise<rm_msgs::PowerManagementSampleAndStatusData>("sample_and_status", 1);
    power_management_initialization_exception_pub_ =
        power_management_nh.advertise<rm_msgs::PowerManagementInitializationExceptionData>("initialization_exception",
                                                                                           1);
    power_management_system_exception_data_ =
        power_management_nh.advertise<rm_msgs::PowerManagementSystemExceptionData>("system_exception", 1);
    power_management_process_stack_overflow_pub_ =
        power_management_nh.advertise<rm_msgs::PowerManagementProcessStackOverflowData>("stack_overflow", 1);
    power_management_unknown_exception_pub_ =
        power_management_nh.advertise<rm_msgs::PowerManagementUnknownExceptionData>("unknown_exception", 1);
    // initSerial
    base_.initSerial();
  };
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher game_robot_status_pub_;
  ros::Publisher game_status_pub_;
  ros::Publisher power_heat_data_pub_;
  ros::Publisher game_robot_hp_pub_;
  ros::Publisher current_sentry_pos_pub_;
  ros::Publisher event_data_pub_;
  ros::Publisher dart_status_pub_;
  ros::Publisher buff_pub_;
  ros::Publisher icra_buff_debuff_zone_status_pub_;
  ros::Publisher supply_projectile_action_pub_;
  ros::Publisher dart_remaining_time_pub_;
  ros::Publisher robot_hurt_pub_;
  ros::Publisher shoot_data_pub_;
  ros::Publisher bullet_allowance_pub_;
  ros::Publisher rfid_status_pub_;
  ros::Publisher dart_client_cmd_pub_;
  ros::Publisher client_map_receive_pub_;
  ros::Publisher robots_position_pub_;
  ros::Publisher radar_mark_pub_;
  ros::Publisher game_robot_pos_pub_;
  ros::Publisher sentry_info_pub_;
  ros::Publisher radar_info_pub_;
  ros::Publisher client_map_send_data_pub_;
  ros::Publisher power_management_sample_and_status_data_pub_;
  ros::Publisher power_management_initialization_exception_pub_;
  ros::Publisher power_management_system_exception_data_;
  ros::Publisher power_management_process_stack_overflow_pub_;
  ros::Publisher power_management_unknown_exception_pub_;

  Base base_;
  std::vector<uint8_t> rx_buffer_;
  rm_referee::RefereeBase referee_ui_;
  int rx_len_;

private:
  int unpack(uint8_t* rx_data);
  void getRobotInfo();
  void publishCapacityData();

  ros::Time last_get_data_time_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};
};

}  // namespace rm_referee
