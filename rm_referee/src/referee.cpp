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
#include "rm_referee/referee.h"

namespace rm_referee
{
// read data from referee
void Referee::read()
{
  if (data_translation_.serial_.available())
  {
    rx_len_ = static_cast<int>(data_translation_.serial_.available());
    data_translation_.serial_.read(rx_buffer_, rx_len_);
  }
  else
  {
    ROS_INFO("Port exception before read");
    return;
  }
  checkUiAdd();
  uint8_t temp_buffer[256] = { 0 };
  int frame_len;
  if (ros::Time::now() - last_get_ > ros::Duration(0.1))
    base_.referee_data_is_online_ = false;
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
  publishCapacityData();
  getRobotInfo();
  clearRxBuffer();
}

int Referee::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  rm_referee::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (data_translation_.verifyCRC8CheckSum(rx_data, k_header_length_) == true)
  {
    if (frame_header.data_length_ > 256)  // temporary and inaccurate value
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length_);
      return 0;
    }
    frame_len = frame_header.data_length_ + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (data_translation_.verifyCRC16CheckSum(rx_data, frame_len) == true)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case rm_referee::RefereeCmdId::GAME_STATUS_CMD:
        {
          rm_referee::GameStatus game_status_ref;
          memcpy(&game_status_ref, rx_data + 7, sizeof(rm_referee::GameStatus));

          base_.game_status_data_.game_type = game_status_ref.game_type_;
          base_.game_status_data_.game_progress = game_status_ref.game_progress_;
          base_.game_status_data_.stage_remain_time = game_status_ref.stage_remain_time_;
          base_.game_status_data_.sync_time_stamp = game_status_ref.sync_time_stamp_;
          base_.game_status_data_.stamp = last_get_;

          game_status_pub_.publish(base_.game_status_data_);
          break;
        }
        case rm_referee::RefereeCmdId::GAME_RESULT_CMD:
        {
          rm_referee::GameResult game_result_ref;
          memcpy(&game_result_ref, rx_data + 7, sizeof(rm_referee::GameResult));
          break;
        }
        case rm_referee::RefereeCmdId::GAME_ROBOT_HP_CMD:
        {
          rm_referee::GameRobotHp game_robot_hp_ref;
          memcpy(&game_robot_hp_ref, rx_data + 7, sizeof(rm_referee::GameRobotHp));

          base_.game_robot_hp_data_.blue_1_robot_hp = game_robot_hp_ref.blue_1_robot_hp_;
          base_.game_robot_hp_data_.blue_2_robot_hp = game_robot_hp_ref.blue_2_robot_hp_;
          base_.game_robot_hp_data_.blue_3_robot_hp = game_robot_hp_ref.blue_3_robot_hp_;
          base_.game_robot_hp_data_.blue_4_robot_hp = game_robot_hp_ref.blue_4_robot_hp_;
          base_.game_robot_hp_data_.blue_5_robot_hp = game_robot_hp_ref.blue_5_robot_hp_;
          base_.game_robot_hp_data_.blue_7_robot_hp = game_robot_hp_ref.blue_7_robot_hp_;
          base_.game_robot_hp_data_.red_1_robot_hp = game_robot_hp_ref.red_1_robot_hp_;
          base_.game_robot_hp_data_.red_2_robot_hp = game_robot_hp_ref.red_2_robot_hp_;
          base_.game_robot_hp_data_.red_3_robot_hp = game_robot_hp_ref.red_3_robot_hp_;
          base_.game_robot_hp_data_.red_4_robot_hp = game_robot_hp_ref.red_4_robot_hp_;
          base_.game_robot_hp_data_.red_5_robot_hp = game_robot_hp_ref.red_5_robot_hp_;
          base_.game_robot_hp_data_.red_7_robot_hp = game_robot_hp_ref.red_7_robot_hp_;
          base_.game_robot_hp_data_.stamp = last_get_;

          game_robot_hp_pub_.publish(base_.game_robot_hp_data_);
          break;
        }
        case rm_referee::RefereeCmdId::DART_STATUS_CMD:
        {
          rm_referee::DartStatus dart_status_ref;
          memcpy(&dart_status_ref, rx_data + 7, sizeof(rm_referee::DartStatus));

          base_.dart_status_data_.dart_belong = dart_status_ref.dart_belong_;
          base_.dart_status_data_.stage_remaining_time = dart_status_ref.stage_remaining_time_;
          base_.dart_status_data_.stamp = last_get_;

          dart_status_pub_.publish(base_.dart_status_data_);
          break;
        }
        case rm_referee::RefereeCmdId::ICRA_ZONE_STATUS_CMD:
        {
          rm_referee::IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status_ref;
          memcpy(&icra_buff_debuff_zone_status_ref, rx_data + 7, sizeof(rm_referee::IcraBuffDebuffZoneStatus));

          base_.icra_buff_debuff_zone_status_data_.blue_1_bullet_left =
              icra_buff_debuff_zone_status_ref.blue_1_bullet_left_;
          base_.icra_buff_debuff_zone_status_data_.blue_2_bullet_left =
              icra_buff_debuff_zone_status_ref.blue_2_bullet_left_;
          base_.icra_buff_debuff_zone_status_data_.red_1_bullet_left =
              icra_buff_debuff_zone_status_ref.red_1_bullet_left_;
          base_.icra_buff_debuff_zone_status_data_.red_2_bullet_left =
              icra_buff_debuff_zone_status_ref.red_2_bullet_left_;
          base_.icra_buff_debuff_zone_status_data_.f_1_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_1_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_1_zone_status = icra_buff_debuff_zone_status_ref.f_1_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.f_2_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_2_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_2_zone_status = icra_buff_debuff_zone_status_ref.f_2_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.f_3_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_3_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_3_zone_status = icra_buff_debuff_zone_status_ref.f_3_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.f_4_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_4_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_4_zone_status = icra_buff_debuff_zone_status_ref.f_4_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.f_5_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_5_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_5_zone_status = icra_buff_debuff_zone_status_ref.f_5_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.f_6_zone_buff_debuff_status =
              icra_buff_debuff_zone_status_ref.f_6_zone_buff_debuff_status_;
          base_.icra_buff_debuff_zone_status_data_.f_6_zone_status = icra_buff_debuff_zone_status_ref.f_6_zone_status_;
          base_.icra_buff_debuff_zone_status_data_.stamp = last_get_;

          icra_buff_debuff_zone_status_pub_.publish(base_.icra_buff_debuff_zone_status_data_);
          break;
        }
        case rm_referee::RefereeCmdId::FIELD_EVENTS_CMD:
        {
          rm_referee::EventData event_ref;
          memcpy(&event_ref, rx_data + 7, sizeof(rm_referee::EventData));

          base_.event_data_.event_data = event_ref.event_type_;
          base_.event_data_.stamp = last_get_;

          event_data_pub_.publish(base_.event_data_);
          break;
        }
        case rm_referee::RefereeCmdId::SUPPLY_PROJECTILE_ACTION_CMD:
        {
          rm_referee::SupplyProjectileAction supply_projectile_action_ref;
          memcpy(&supply_projectile_action_ref, rx_data + 7, sizeof(rm_referee::SupplyProjectileAction));

          base_.supply_projectile_action_data_.supply_projectile_id =
              supply_projectile_action_ref.supply_projectile_id_;
          base_.supply_projectile_action_data_.supply_projectile_num =
              supply_projectile_action_ref.supply_projectile_num_;
          base_.supply_projectile_action_data_.supply_projectile_step =
              supply_projectile_action_ref.supply_projectile_step_;
          base_.supply_projectile_action_data_.supply_robot_id = supply_projectile_action_ref.supply_robot_id_;
          base_.supply_projectile_action_data_.stamp = last_get_;

          supply_projectile_action_pub_.publish(base_.supply_projectile_action_data_);
          break;
        }
        case rm_referee::RefereeCmdId::REFEREE_WARNING_CMD:
        {
          rm_referee::RefereeWarning referee_warning_ref;
          memcpy(&referee_warning_ref, rx_data + 7, sizeof(rm_referee::RefereeWarning));
          break;
        }
        case rm_referee::RefereeCmdId::DART_REMAINING_CMD:
        {
          rm_referee::DartRemainingTime dart_remaining_time_ref;
          memcpy(&dart_remaining_time_ref, rx_data + 7, sizeof(rm_referee::DartRemainingTime));

          base_.dart_remaining_time_data_.dart_remaining_time = dart_remaining_time_ref.dart_remaining_time_;
          base_.dart_remaining_time_data_.stamp = last_get_;

          dart_remaining_time_pub_.publish(base_.dart_remaining_time_data_);
          break;
        }
        case rm_referee::RefereeCmdId::ROBOT_STATUS_CMD:
        {
          rm_referee::GameRobotStatus game_robot_status_ref;
          memcpy(&game_robot_status_ref, rx_data + 7, sizeof(rm_referee::GameRobotStatus));

          base_.game_robot_status_data_.mains_power_chassis_output = game_robot_status_ref.mains_power_chassis_output_;
          base_.game_robot_status_data_.mains_power_gimbal_output = game_robot_status_ref.mains_power_gimbal_output_;
          base_.game_robot_status_data_.mains_power_shooter_output = game_robot_status_ref.mains_power_shooter_output_;
          base_.game_robot_status_data_.chassis_power_limit = game_robot_status_ref.chassis_power_limit_;
          base_.game_robot_status_data_.shooter_id_1_17_mm_cooling_limit =
              game_robot_status_ref.shooter_id_1_17_mm_cooling_limit_;
          base_.game_robot_status_data_.shooter_id_1_17_mm_cooling_rate =
              game_robot_status_ref.shooter_id_1_17_mm_cooling_rate_;
          base_.game_robot_status_data_.shooter_id_2_17_mm_cooling_limit =
              game_robot_status_ref.shooter_id_2_17_mm_cooling_limit_;
          base_.game_robot_status_data_.shooter_id_2_17_mm_cooling_rate =
              game_robot_status_ref.shooter_id_2_17_mm_cooling_rate_;
          base_.game_robot_status_data_.shooter_id_1_42_mm_cooling_limit =
              game_robot_status_ref.shooter_id_1_42_mm_cooling_limit_;
          base_.game_robot_status_data_.shooter_id_1_42_mm_cooling_rate =
              game_robot_status_ref.shooter_id_1_42_mm_cooling_rate_;
          base_.game_robot_status_data_.shooter_id_1_17_mm_speed_limit =
              game_robot_status_ref.shooter_id_1_17_mm_speed_limit_;
          base_.game_robot_status_data_.shooter_id_2_17_mm_speed_limit =
              game_robot_status_ref.shooter_id_2_17_mm_speed_limit_;
          base_.game_robot_status_data_.shooter_id_1_42_mm_speed_limit =
              game_robot_status_ref.shooter_id_1_42_mm_speed_limit_;
          base_.game_robot_status_data_.robot_id = game_robot_status_ref.robot_id_;
          base_.game_robot_status_data_.robot_level = game_robot_status_ref.robot_level_;
          base_.game_robot_status_data_.stamp = last_get_;

          base_.referee_pub_data_.is_online = base_.referee_data_is_online_;

          base_.referee_pub_data_.stamp = last_get_;

          referee_ui_->robotStatusDataCallBack(base_.game_robot_status_data_, last_get_);

          game_robot_status_pub_.publish(base_.game_robot_status_data_);
          referee_pub_.publish(base_.referee_pub_data_);
          break;
        }
        case rm_referee::RefereeCmdId::POWER_HEAT_DATA_CMD:
        {
          rm_referee::PowerHeatData power_heat_ref;
          memcpy(&power_heat_ref, rx_data + 7, sizeof(rm_referee::PowerHeatData));

          base_.power_heat_data_.chassis_power_buffer = power_heat_ref.chassis_power_buffer_;
          base_.power_heat_data_.chassis_power = power_heat_ref.chassis_power_;
          base_.power_heat_data_.shooter_id_1_17_mm_cooling_heat = power_heat_ref.shooter_id_1_17_mm_cooling_heat_;
          base_.power_heat_data_.shooter_id_2_17_mm_cooling_heat = power_heat_ref.shooter_id_2_17_mm_cooling_heat_;
          base_.power_heat_data_.shooter_id_1_42_mm_cooling_heat = power_heat_ref.shooter_id_1_42_mm_cooling_heat_;
          base_.power_heat_data_.chassis_volt = static_cast<uint16_t>(power_heat_ref.chassis_volt_ * 0.001);  // mV->V
          base_.power_heat_data_.chassis_current =
              static_cast<uint16_t>(power_heat_ref.chassis_current_ * 0.001);  // mA->A

          base_.referee_pub_data_.is_online = base_.referee_data_is_online_;
          base_.referee_pub_data_.stamp = last_get_;
          base_.power_heat_data_.stamp = last_get_;

          referee_pub_.publish(base_.referee_pub_data_);
          power_heat_data_pub_.publish(base_.power_heat_data_);
          break;
        }
        case rm_referee::RefereeCmdId::ROBOT_POS_CMD:
        {
          rm_referee::GameRobotPos game_robot_pos_ref;
          memcpy(&game_robot_pos_ref, rx_data + 7, sizeof(rm_referee::GameRobotPos));
          break;
        }
        case rm_referee::RefereeCmdId::BUFF_CMD:
        {
          rm_referee::Buff referee_buff;
          memcpy(&referee_buff, rx_data + 7, sizeof(rm_referee::Buff));
          break;
        }
        case rm_referee::RefereeCmdId::AERIAL_ROBOT_ENERGY_CMD:
        {
          rm_referee::AerialRobotEnergy aerial_robot_energy_ref;
          memcpy(&aerial_robot_energy_ref, rx_data + 7, sizeof(rm_referee::AerialRobotEnergy));
          break;
        }
        case rm_referee::RefereeCmdId::ROBOT_HURT_CMD:
        {
          rm_referee::RobotHurt robot_hurt_ref;
          memcpy(&robot_hurt_ref, rx_data + 7, sizeof(rm_referee::RobotHurt));

          base_.robot_hurt_data_.armor_id = robot_hurt_ref.armor_id_;
          base_.robot_hurt_data_.hurt_type = robot_hurt_ref.hurt_type_;
          base_.robot_hurt_data_.stamp = last_get_;

          base_.referee_pub_data_.is_online = base_.referee_data_is_online_;
          base_.referee_pub_data_.stamp = last_get_;

          referee_ui_->robotHurtDataCallBack(base_.robot_hurt_data_, last_get_);

          robot_hurt_pub_.publish(base_.robot_hurt_data_);
          referee_pub_.publish(base_.referee_pub_data_);
          break;
        }
        case rm_referee::RefereeCmdId::SHOOT_DATA_CMD:
        {
          rm_referee::ShootData shoot_data_ref;
          memcpy(&shoot_data_ref, rx_data + 7, sizeof(rm_referee::ShootData));

          base_.shoot_data_.bullet_freq = shoot_data_ref.bullet_freq_;
          base_.shoot_data_.bullet_speed = shoot_data_ref.bullet_speed_;
          base_.shoot_data_.bullet_type = shoot_data_ref.bullet_type_;
          base_.shoot_data_.shooter_id = shoot_data_ref.shooter_id_;
          base_.shoot_data_.stamp = last_get_;

          base_.referee_pub_data_.is_online = base_.referee_data_is_online_;
          base_.referee_pub_data_.stamp = last_get_;

          shoot_data_pub_.publish(base_.shoot_data_);
          referee_pub_.publish(base_.referee_pub_data_);
          break;
        }
        case rm_referee::RefereeCmdId::BULLET_REMAINING_CMD:
        {
          rm_referee::BulletRemaining bullet_remaining_ref;
          memcpy(&bullet_remaining_ref, rx_data + 7, sizeof(rm_referee::BulletRemaining));

          base_.bullet_remaining_data_.bullet_remaining_num_17_mm = bullet_remaining_ref.bullet_remaining_num_17_mm_;
          base_.bullet_remaining_data_.bullet_remaining_num_42_mm = bullet_remaining_ref.bullet_remaining_num_42_mm_;
          base_.bullet_remaining_data_.coin_remaining_num = bullet_remaining_ref.coin_remaining_num_;
          base_.bullet_remaining_data_.stamp = last_get_;

          bullet_remaining_pub_.publish(base_.bullet_remaining_data_);
          break;
        }
        case rm_referee::RefereeCmdId::ROBOT_RFID_STATUS_CMD:
        {
          rm_referee::RfidStatus rfid_status_ref;
          memcpy(&rfid_status_ref, rx_data + 7, sizeof(rm_referee::RfidStatus));

          base_.rfid_status_data_.rfid_status = rfid_status_ref.rfid_status_;
          base_.rfid_status_data_.stamp = last_get_;

          rfid_status_pub_.publish(base_.rfid_status_data_);
          break;
        }
        case rm_referee::RefereeCmdId::DART_CLIENT_CMD:
        {
          rm_referee::DartClientCmd dart_client_cmd_ref;
          memcpy(&dart_client_cmd_ref, rx_data + 7, sizeof(rm_referee::DartClientCmd));

          base_.dart_client_cmd_data_.dart_attack_target = dart_client_cmd_ref.dart_attack_target_;
          base_.dart_client_cmd_data_.dart_launch_opening_status = dart_client_cmd_ref.dart_launch_opening_status_;
          base_.dart_client_cmd_data_.first_dart_speed = dart_client_cmd_ref.first_dart_speed_;
          base_.dart_client_cmd_data_.second_dart_speed = dart_client_cmd_ref.second_dart_speed_;
          base_.dart_client_cmd_data_.third_dart_speed = dart_client_cmd_ref.third_dart_speed_;
          base_.dart_client_cmd_data_.fourth_dart_speed = dart_client_cmd_ref.fourth_dart_speed_;
          base_.dart_client_cmd_data_.last_dart_launch_time = dart_client_cmd_ref.last_dart_launch_time_;
          base_.dart_client_cmd_data_.operate_launch_cmd_time = dart_client_cmd_ref.operate_launch_cmd_time_;
          base_.dart_client_cmd_data_.target_change_time = dart_client_cmd_ref.target_change_time_;
          base_.dart_client_cmd_data_.stamp = last_get_;

          dart_client_cmd_pub_.publish(base_.dart_client_cmd_data_);
          break;
        }
        case rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD:
        {
          rm_referee::InteractiveData interactive_data_ref;  // local variable temporarily before moving referee data
          memcpy(&interactive_data_ref, rx_data + 7, sizeof(rm_referee::InteractiveData));
          break;
        }
        default:
          ROS_WARN("Referee command ID not found.");
          break;
      }
      base_.referee_data_is_online_ = true;
      last_get_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

void Referee::getRobotInfo()
{
  data_translation_.robot_id_ = base_.game_robot_status_data_.robot_id;
  data_translation_.robot_color_ = base_.game_robot_status_data_.robot_id >= 100 ? "blue" : "red";
  if (data_translation_.robot_id_ != rm_referee::RobotId::BLUE_SENTRY &&
      data_translation_.robot_id_ != rm_referee::RobotId::RED_SENTRY)
  {
    switch (data_translation_.robot_id_)
    {
      case rm_referee::RobotId::BLUE_HERO:
        data_translation_.client_id_ = rm_referee::ClientId::BLUE_HERO_CLIENT;
        break;
      case rm_referee::RobotId::BLUE_ENGINEER:
        data_translation_.client_id_ = rm_referee::ClientId::BLUE_ENGINEER_CLIENT;
        break;
      case rm_referee::RobotId::BLUE_STANDARD_3:
        data_translation_.client_id_ = rm_referee::ClientId::BLUE_STANDARD_3_CLIENT;
        break;
      case rm_referee::RobotId::BLUE_STANDARD_4:
        data_translation_.client_id_ = rm_referee::ClientId::BLUE_STANDARD_4_CLIENT;
        break;
      case rm_referee::RobotId::BLUE_STANDARD_5:
        data_translation_.client_id_ = rm_referee::ClientId::BLUE_STANDARD_5_CLIENT;
        break;
      case rm_referee::RobotId::RED_HERO:
        data_translation_.client_id_ = rm_referee::ClientId::RED_HERO_CLIENT;
        break;
      case rm_referee::RobotId::RED_ENGINEER:
        data_translation_.client_id_ = rm_referee::ClientId::RED_ENGINEER_CLIENT;
        break;
      case rm_referee::RobotId::RED_STANDARD_3:
        data_translation_.client_id_ = rm_referee::ClientId::RED_STANDARD_3_CLIENT;
        break;
      case rm_referee::RobotId::RED_STANDARD_4:
        data_translation_.client_id_ = rm_referee::ClientId::RED_STANDARD_4_CLIENT;
        break;
      case rm_referee::RobotId::RED_STANDARD_5:
        data_translation_.client_id_ = rm_referee::ClientId::RED_STANDARD_5_CLIENT;
        break;
    }
  }
}

void Referee::publishCapacityData()
{
  base_.super_capacitor_data_.capacity = static_cast<float>(base_.capacity_data_ref_.cap_power);
  base_.super_capacitor_data_.chassis_power_buffer = static_cast<uint16_t>(base_.capacity_data_ref_.buffer_power);
  base_.super_capacitor_data_.limit_power = static_cast<float>(base_.capacity_data_ref_.limit_power);
  base_.super_capacitor_data_.chassis_power = static_cast<float>(base_.capacity_data_ref_.chassis_power);
  base_.super_capacitor_data_.stamp = super_capacitor_.last_get_data_;

  base_.capacity_data_.buffer_power = base_.capacity_data_ref_.buffer_power;
  base_.capacity_data_.is_online = base_.capacity_data_ref_.is_online;
  base_.capacity_data_.cap_power = base_.capacity_data_ref_.cap_power;
  base_.capacity_data_.chassis_power = base_.capacity_data_ref_.chassis_power;
  base_.capacity_data_.limit_power = base_.capacity_data_ref_.limit_power;
  base_.capacity_data_.stamp = last_get_;

  referee_ui_->capacityDataCallBack(base_.capacity_data_, last_get_);

  super_capacitor_pub_.publish(base_.super_capacitor_data_);
  capacity_data_pub_.publish(base_.capacity_data_);
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
    if (count >= static_cast<int>(sizeof(receive_buffer_)))
    {
      memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
      memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
      receive_buf_counter_ = 0;
    }
  }
  if (data_.chassis_power >= 120.)
    data_.chassis_power = 120.;
  if (data_.chassis_power <= 0.)
    data_.chassis_power = 0.;
  if (data_.buffer_power >= 25.)
    data_.buffer_power = 25.;
  if (data_.buffer_power <= 0.)
    data_.buffer_power = 0.;
  if (data_.cap_power >= 1.)
    data_.cap_power = 1.;
  if (ros::Time::now() - last_get_data_ > ros::Duration(0.1))
    data_.is_online = false;
}

void SuperCapacitor::receiveCallBack(unsigned char package_id, const unsigned char* data)
{
  if (package_id == 0)
  {
    last_get_data_ = ros::Time::now();
    data_.is_online = true;
    data_.chassis_power = static_cast<double>(int16ToFloat((data[0] << 8) | data[1]));
    data_.limit_power = static_cast<double>(int16ToFloat((data[2] << 8) | data[3]));
    data_.buffer_power = static_cast<double>(int16ToFloat((data[4] jjjjjj << 8) | data[5]));
    data_.cap_power = static_cast<double>(int16ToFloat((data[6] << 8) | data[7]));
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
  fp32 = reinterpret_cast<float*>(&fInt32);
  return *fp32;
}
}  // namespace rm_referee
