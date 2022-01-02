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
// Created by qiayuan on 1/20/21.
//

#pragma once

#include <string>
#include <rm_common/filters/lp_filter.h>
#include <unordered_map>

namespace rm_hw
{
struct ActCoeff
{
  double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
      act2effort_offset, kp2act, kd2act;  // for MIT Cheetah motor
};

struct ActData
{
  std::string name;
  std::string type;
  ros::Time stamp;
  uint64_t seq;
  bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
  uint16_t q_raw;
  int16_t qd_raw;
  uint8_t temp;
  int64_t q_circle;
  uint16_t q_last;
  double frequency;
  double pos, vel, effort;
  double cmd_pos, cmd_vel, cmd_effort, exe_effort;
  double offset;
  // For multiple cycle under absolute encoder (RoboMaster motor)
  LowPassFilter* lp_filter;
};

struct ImuData
{
  double ori[4];
  double angular_vel[3], linear_acc[3];
  double ori_cov[9], angular_vel_cov[9], linear_acc_cov[9];
  double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
  bool accel_updated, gyro_updated, camera_trigger;
};

struct TofData
{
  double distance;
  int dis_status;
  double signal_strength;
};

struct CanDataPtr
{
  std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
  std::unordered_map<int, ActData>* id2act_data_;
  std::unordered_map<int, ImuData>* id2imu_data_;
  std::unordered_map<int, TofData>* id2tof_data_;
};
}  // namespace rm_hw
