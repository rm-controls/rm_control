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
// Created by bruce on 2021/7/28.
//

#pragma once

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/CapacityData.h>

namespace rm_common
{
class PowerLimit
{
public:
  PowerLimit(ros::NodeHandle& nh, const rm_msgs::ChassisCmd& chassis_cmd) : chassis_cmd_(chassis_cmd)

  {
    if (!nh.getParam("safety_power", safety_power_))
      ROS_ERROR("Safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("capacitor_threshold", capacitor_threshold_))
      ROS_ERROR("Capacitor threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("charge_power", charge_power_))
      ROS_ERROR("Charge power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("extra_power", extra_power_))
      ROS_ERROR("Extra power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_power", burst_power_))
      ROS_ERROR("Burst power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("power_gain", power_gain_))
      ROS_ERROR("power gain no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("buffer_threshold", buffer_threshold_))
      ROS_ERROR("buffer threshold no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  typedef enum
  {
    TEST = 0,
    BURST = 1,
    NORMAL = 2,
    CHARGE = 3,
  } Mode;

  void updateState(uint8_t state)
  {
    state_ = state;
  }
  void setRobotId(int robot_id)
  {
    robot_id_ = robot_id;
  }
  void setGameProgress(int game_progress)
  {
    game_progress_ = game_progress;
  }
  void setChassisPowerLimit(int chassis_power_limit)
  {
    chassis_power_limit_ = chassis_power_limit;
  }
  void setChassisPowerBuffer(int chassis_power_buffer)
  {
    chassis_power_buffer_ = chassis_power_buffer;
  }
  void setCapPower(float cap_power)
  {
    cap_power_ = cap_power;
  }
  void setCapacityStatus(bool status)
  {
    capacity_is_online_ = status;
  }
  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }

  uint8_t getState()
  {
    return state_;
  }
  double getLimitPower()
  {
    if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_SENTRY || robot_id_ == rm_msgs::GameRobotStatus::RED_SENTRY)
      limit_power_ = 30;
    else if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER)
      limit_power_ = 400;
    else
    {  // standard and hero
      if (referee_is_online_)
      {
        if (capacity_is_online_)
        {
          if (game_progress_ == 1)
            return 30;  // calibra
          if (chassis_power_limit_ > 120)
            limit_power_ = burst_power_;
          else
          {
            switch (state_)
            {
              case TEST:
                test();
                break;
              case BURST:
                burst();
                break;
              case NORMAL:
                normal();
                break;
              case CHARGE:
                charge();
                break;
            }
            if (state_ != Mode::BURST && (abs(limit_power_ - chassis_power_limit_) < 0.05))
              normal();
          }
        }
        else
          limit_power_ = chassis_power_limit_;
      }
      else
        limit_power_ = safety_power_;
    }
    return limit_power_;
  }

private:
  void charge()
  {
    limit_power_ = chassis_power_limit_ * 0.85;
  }
  void normal()
  {
    double buffer_energy_error = chassis_power_buffer_ - buffer_threshold_;
    double plus_power = buffer_energy_error * power_gain_;
    limit_power_ = chassis_power_limit_ + plus_power;
  }
  void test()
  {
    limit_power_ = 0.0;
  }
  void burst()
  {
    if (cap_power_ > capacitor_threshold_)
    {
      if (chassis_cmd_.mode == rm_msgs::ChassisCmd::GYRO)
        limit_power_ = chassis_power_limit_ + extra_power_;
      else
        limit_power_ = burst_power_;
    }
    else
      limit_power_ = chassis_power_limit_;
  }

  int game_progress_;
  int chassis_power_buffer_;
  int robot_id_, chassis_power_limit_;
  float cap_power_;
  double limit_power_;
  double safety_power_{};
  double capacitor_threshold_{};
  double charge_power_{}, extra_power_{}, burst_power_{};
  double buffer_threshold_{};
  double power_gain_{};
  uint8_t state_{};

  const rm_msgs::ChassisCmd& chassis_cmd_;
  bool referee_is_online_;
  bool capacity_is_online_;
};
}  // namespace rm_common
