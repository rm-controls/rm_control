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
// Created by qiayuan on 1/5/21.
//

#pragma once

#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/LpData.h>

class LowPassFilter
{
public:
  explicit LowPassFilter(ros::NodeHandle& nh);
  explicit LowPassFilter(double cutoff_freq);
  void input(double in);
  void input(double in, ros::Time time);
  double output();
  void reset();

private:
  double in_[3]{};
  double out_[3]{};

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_ = -1;
  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at 1/4 of the sample rate.
  double c_ = 1.;
  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;
  bool is_debug_{};

  ros::Time prev_time_;
  ros::Duration delta_t_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LpData>> realtime_pub_{};
};
