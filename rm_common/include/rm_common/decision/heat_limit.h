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

#include <ros/ros.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/ShootCmd.h>

namespace rm_common
{
class HeatLimit
{
public:
  HeatLimit(ros::NodeHandle& nh)

  {
    if (!nh.getParam("low_shoot_frequency", low_shoot_frequency_))
      ROS_ERROR("Low shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("high_shoot_frequency", high_shoot_frequency_))
      ROS_ERROR("High shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_shoot_frequency", burst_shoot_frequency_))
      ROS_ERROR("Burst shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("minimal_shoot_frequency", minimal_shoot_frequency_))
      ROS_ERROR("Minimal shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("heat_coeff", heat_coeff_))
      ROS_ERROR("Heat coeff no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("type", type_))
      ROS_ERROR("Shooter type no defined (namespace: %s)", nh.getNamespace().c_str());
    //    nh.param("safe_speed_limit", shooter_speed_limit_, 15);
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
    MINIMAL = 3
  } ShootHz;

  void setStatusOfShooter(const rm_msgs::GameRobotStatus data)
  {
    shooter_cooling_limit_ = data.shooter_cooling_limit;
    shooter_cooling_rate_ = data.shooter_cooling_rate;
  }

  void setCoolingHeatOfShooter(const rm_msgs::PowerHeatData data)
  {
    if (type_ == "ID1_17MM")
    {
      shooter_cooling_heat_ = data.shooter_id_1_17_mm_cooling_heat;
    }
    else if (type_ == "ID2_17MM")
    {
      shooter_cooling_heat_ = data.shooter_id_2_17_mm_cooling_heat;
    }
    else if (type_ == "ID1_42MM")
    {
      shooter_cooling_heat_ = data.shooter_id_1_42_mm_cooling_heat;
    }
  }

  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }

  double getShootFrequency() const
  {
    if (state_ == BURST)
      return shoot_frequency_;
    if (!referee_is_online_)
      return safe_shoot_frequency_;

    if (shooter_cooling_limit_ - shooter_cooling_heat_ < bullet_heat_)
      return 0.0;
    else if (shooter_cooling_limit_ - shooter_cooling_heat_ == bullet_heat_)
      return shooter_cooling_rate_ / bullet_heat_;
    else if (shooter_cooling_limit_ - shooter_cooling_heat_ <= bullet_heat_ * heat_coeff_)
      return (shooter_cooling_limit_ - shooter_cooling_heat_) / (bullet_heat_ * heat_coeff_) *
                 (shoot_frequency_ - shooter_cooling_rate_ / bullet_heat_) +
             shooter_cooling_rate_ / bullet_heat_;
    else
      return shoot_frequency_;
  }

  int getSpeedLimit()
  {
    updateExpectShootFrequency();
    if (type_ == "ID1_17MM")
      return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
    else if (type_ == "ID2_17MM")
      return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
    else if (type_ == "ID1_42MM")
      return rm_msgs::ShootCmd::SPEED_16M_PER_SECOND;
    return -1;  // TODO unsafe!
  }

  int getCoolingLimit()
  {
    return shooter_cooling_limit_;
  }

  int getCoolingHeat()
  {
    return shooter_cooling_heat_;
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
      shoot_frequency_ = burst_shoot_frequency_;
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
    else if (state_ == HeatLimit::MINIMAL)
    {
      shoot_frequency_ = minimal_shoot_frequency_;
      burst_flag_ = false;
    }
    else
    {
      shoot_frequency_ = safe_shoot_frequency_;
      burst_flag_ = false;
    }
  }

  uint8_t state_{};
  std::string type_{};
  bool burst_flag_ = false;
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{}, shoot_frequency_{}, low_shoot_frequency_{},
      high_shoot_frequency_{}, burst_shoot_frequency_{}, minimal_shoot_frequency_{};

  bool referee_is_online_;
  int shooter_cooling_limit_, shooter_cooling_rate_, shooter_cooling_heat_;
};

}  // namespace rm_common
