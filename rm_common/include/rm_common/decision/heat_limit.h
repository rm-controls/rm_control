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
// Created by qiayuan on 5/19/21.
//

#pragma once

#include "rm_common/referee/data.h"
namespace rm_common
{
class HeatLimit
{
public:
  HeatLimit(ros::NodeHandle& nh, const RefereeData& referee_data) : referee_data_(referee_data)
  {
    if (!nh.getParam("low_shoot_frequency", low_shoot_frequency_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("high_shoot_frequency", high_shoot_frequency_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_shoot_frequency", burst_shoot_frequency_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("heat_coeff", heat_coeff_))
      ROS_ERROR("Safe shoot heat coeff frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("type", type_))
      ROS_ERROR("Shooter type no defined (namespace: %s)", nh.getNamespace().c_str());
    if (type_ == "ID1_42MM")
      bullet_heat_ = 100.;
    else
      bullet_heat_ = 10.;
  }

  typedef enum
  {
    LOW = 0,
    HIGH = 1,
    BURST = 2,
  } ShootHz;

  double getShootFrequency() const
  {
    if (state_ == BURST)
      return shoot_frequency_;
    if (!referee_data_.is_online_)
      return safe_shoot_frequency_;
    double cooling_limit{}, cooling_rate{}, cooling_heat{};
    if (type_ == "ID1_17MM")
    {
      cooling_limit = referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_;
      cooling_rate = referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_rate_;
      cooling_heat = referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_;
    }
    else if (type_ == "ID2_17MM")
    {
      cooling_limit = referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_limit_;
      cooling_rate = referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_rate_;
      cooling_heat = referee_data_.power_heat_data_.shooter_id_2_17_mm_cooling_heat_;
    }
    else if (type_ == "ID1_42MM")
    {
      cooling_limit = referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_;
      cooling_rate = referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_rate_;
      cooling_heat = referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_;
    }

    if (cooling_limit - cooling_heat < bullet_heat_)
      return 0.0;
    else if (cooling_limit - cooling_heat == bullet_heat_)
      return cooling_rate / bullet_heat_;
    else if (cooling_limit - cooling_heat <= bullet_heat_ * heat_coeff_)
      return (cooling_limit - cooling_heat) / (bullet_heat_ * heat_coeff_) *
                 (shoot_frequency_ - cooling_rate / bullet_heat_) +
             cooling_rate / bullet_heat_;
    else
      return shoot_frequency_;
  }

  int getSpeedLimit()
  {
    updateExpectShootFrequency();
    if (type_ == "ID1_17MM")
      switch (referee_data_.game_robot_status_.shooter_id_1_17_mm_speed_limit_)
      {
        case 15:
          return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
        case 18:
          return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
        case 30:
          return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
        default:
          return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;  // Safety speed
      }
    else if (type_ == "ID2_17MM")
      switch (referee_data_.game_robot_status_.shooter_id_2_17_mm_speed_limit_)
      {
        case 15:
          return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
        case 18:
          return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
        case 30:
          return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
        default:
          return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;  // Safety speed
      }
    else if (type_ == "ID1_42MM")
      switch (referee_data_.game_robot_status_.shooter_id_1_42_mm_speed_limit_)
      {
        case 10:
          return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;
        case 16:
          return rm_msgs::ShootCmd::SPEED_16M_PER_SECOND;
        default:
          return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;  // Safety speed
      }
    return -1;  // TODO unsafe!
  }

  void setShootFrequency(uint8_t mode)
  {
    state_ = mode;
  }

  bool getShootFrequencyMode() const
  {
    return state_;
  }

private:
  void updateExpectShootFrequency()
  {
    if (state_ == HeatLimit::BURST)
    {
      shoot_frequency_ = high_shoot_frequency_;
      burst_flag_ = true;
    }
    else if (state_ == HeatLimit::LOW)
    {
      shoot_frequency_ = low_shoot_frequency_;
      burst_flag_ = false;
    }
    else if (state_ == HeatLimit::HIGH)
    {
      shoot_frequency_ = high_shoot_frequency_;
      burst_flag_ = false;
    }
    else
    {
      shoot_frequency_ = safe_shoot_frequency_;
      burst_flag_ = false;
    }
  }

  std::string type_{};
  const RefereeData& referee_data_;
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{}, shoot_frequency_{}, low_shoot_frequency_{},
      high_shoot_frequency_{}, burst_shoot_frequency_{};
  uint8_t state_{};
  bool burst_flag_ = false;
};

}  // namespace rm_common
