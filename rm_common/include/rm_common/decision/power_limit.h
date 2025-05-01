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
#include <rm_msgs/PowerManagementSampleAndStatusData.h>

namespace rm_common
{
class PowerLimit
{
public:
  PowerLimit(ros::NodeHandle& nh)

  {
    if (!nh.getParam("safety_power", safety_power_))
      ROS_ERROR("Safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("capacitor_threshold", capacitor_threshold_))
      ROS_ERROR("Capacitor threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("disable_cap_gyro_threshold", disable_cap_gyro_threshold_))
      ROS_ERROR("Disable cap gyro threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("enable_cap_gyro_threshold", enable_cap_gyro_threshold_))
      ROS_ERROR("Enable cap gyro threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("charge_power", charge_power_))
      ROS_ERROR("Charge power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("extra_power", extra_power_))
      ROS_ERROR("Extra power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_power", burst_power_))
      ROS_ERROR("Burst power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("standard_power", standard_power_))
      ROS_ERROR("Standard power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_power_limit", max_power_limit_))
      ROS_ERROR("max power limit no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("power_gain", power_gain_))
      ROS_ERROR("power gain no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("buffer_threshold", buffer_threshold_))
      ROS_ERROR("buffer threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("is_new_capacitor", is_new_capacitor_))
      ROS_ERROR("is_new_capacitor no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("total_burst_time", total_burst_time_))
      ROS_ERROR("total burst time no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  typedef enum
  {
    CHARGE = 0,
    BURST = 1,
    NORMAL = 2,
    ALLOFF = 3,
    TEST = 4,
  } Mode;

  void updateSafetyPower(int safety_power)
  {
    if (safety_power > 0)
      safety_power_ = safety_power;
    ROS_INFO("update safety power: %d", safety_power);
  }
  void updateState(uint8_t state)
  {
    if (!capacitor_is_on_)
      expect_state_ = ALLOFF;
    else
      expect_state_ = state;
  }
  void updateCapSwitchState(bool state)
  {
    capacitor_is_on_ = state;
  }
  void setGameRobotData(const rm_msgs::GameRobotStatus data)
  {
    robot_id_ = data.robot_id;
    chassis_power_limit_ = data.chassis_power_limit;
  }
  void setChassisPowerBuffer(const rm_msgs::PowerHeatData data)
  {
    chassis_power_buffer_ = data.chassis_power_buffer;
    power_buffer_threshold_ = chassis_power_buffer_ * 0.8;
  }
  void setCapacityData(const rm_msgs::PowerManagementSampleAndStatusData data)
  {
    capacity_is_online_ = ros::Time::now() - data.stamp < ros::Duration(0.3);
    cap_energy_ = data.capacity_remain_charge;
    cap_state_ = data.state_machine_running_state;
  }
  void setRefereeStatus(bool status)
  {
    referee_is_online_ = status;
  }
  void setStartBurstTime(const ros::Time start_burst_time)
  {
    start_burst_time_ = start_burst_time;
  }
  ros::Time getStartBurstTime() const
  {
    return start_burst_time_;
  }
  uint8_t getState()
  {
    return expect_state_;
  }
  void setGyroPower(rm_msgs::ChassisCmd& chassis_cmd)
  {
    if (!allow_gyro_cap_ && cap_energy_ >= enable_cap_gyro_threshold_)
      allow_gyro_cap_ = true;
    if (allow_gyro_cap_ && cap_energy_ <= disable_cap_gyro_threshold_)
      allow_gyro_cap_ = false;
    if (allow_gyro_cap_ && chassis_power_limit_ < 80)
      chassis_cmd.power_limit = chassis_power_limit_ + extra_power_;
    else
      expect_state_ = NORMAL;
  }
  void setLimitPower(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER)
      chassis_cmd.power_limit = 400;
    else
    {  // standard and hero
      if (referee_is_online_)
      {
        if (capacity_is_online_ && expect_state_ != ALLOFF)
        {
          if (chassis_power_limit_ > burst_power_)
            chassis_cmd.power_limit = burst_power_;
          else
          {
            switch (is_new_capacitor_ ? expect_state_ : cap_state_)
            {
              case NORMAL:
                normal(chassis_cmd);
                break;
              case BURST:
                burst(chassis_cmd, is_gyro);
                break;
              case CHARGE:
                charge(chassis_cmd);
                break;
              default:
                zero(chassis_cmd);
                break;
            }
          }
        }
        else
          normal(chassis_cmd);
      }
      else
        chassis_cmd.power_limit = safety_power_;
    }
  }

private:
  void charge(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = chassis_power_limit_ * 0.70;
  }
  void normal(rm_msgs::ChassisCmd& chassis_cmd)
  {
    double buffer_energy_error = chassis_power_buffer_ - buffer_threshold_;
    double plus_power = buffer_energy_error * power_gain_;
    chassis_cmd.power_limit = chassis_power_limit_ + plus_power;
    // TODO:Add protection when buffer<5
    if (chassis_cmd.power_limit > max_power_limit_)
      chassis_cmd.power_limit = max_power_limit_;
  }
  void zero(rm_msgs::ChassisCmd& chassis_cmd)
  {
    chassis_cmd.power_limit = 0.0;
  }
  void burst(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro)
  {
    if (cap_state_ != ALLOFF && cap_energy_ > capacitor_threshold_ && chassis_power_buffer_ > power_buffer_threshold_)
    {
      if (is_gyro)
        setGyroPower(chassis_cmd);
      else if (ros::Time::now() - start_burst_time_ < ros::Duration(total_burst_time_))
        chassis_cmd.power_limit = burst_power_;
      else
        chassis_cmd.power_limit = standard_power_;
    }
    else
      expect_state_ = NORMAL;
  }

  int chassis_power_buffer_;
  int robot_id_, chassis_power_limit_;
  int max_power_limit_{ 70 };
  float cap_energy_;
  double safety_power_{};
  double capacitor_threshold_{};
  double power_buffer_threshold_{ 50.0 };
  double enable_cap_gyro_threshold_{}, disable_cap_gyro_threshold_{};
  double charge_power_{}, extra_power_{}, burst_power_{}, standard_power_{};
  double buffer_threshold_{};
  double power_gain_{};
  bool is_new_capacitor_{ false };
  uint8_t expect_state_{}, cap_state_{};
  bool capacitor_is_on_{ true };
  bool allow_gyro_cap_{ false };

  ros::Time start_burst_time_{};
  int total_burst_time_{};

  bool referee_is_online_{ false };
  bool capacity_is_online_{ false };
};
}  // namespace rm_common
