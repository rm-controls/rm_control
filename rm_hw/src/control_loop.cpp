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
// Created by qiayuan on 12/30/20.
//
#include "rm_hw/control_loop.h"

rm_hw::RmRobotHWLoop::RmRobotHWLoop(ros::NodeHandle& nh, std::shared_ptr<RmRobotHW> hardware_interface)
  : nh_(nh), hardware_interface_(std::move(hardware_interface))
{
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Load rosparams
  int error = 0;
  ros::NodeHandle nh_p("~");
  error += !nh_p.getParam("loop_frequency", loop_hz_);
  error += !nh_p.getParam("cycle_time_error_threshold", cycle_time_error_threshold_);
  if (error > 0)
  {
    char error_message[] =
        "could not retrieve one of the required parameters\n\trm_hw/loop_hz or rm_hw/cycle_time_error_threshold";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  last_time_ = steady_clock::now();

  // Start timer that will periodically call RmRobotHWLoop::update
  desired_update_freq_ = ros::Duration(1 / loop_hz_);
  loop_timer_ = nh_.createTimer(desired_update_freq_, &RmRobotHWLoop::update, this);
}

void rm_hw::RmRobotHWLoop::update(const ros::TimerEvent&)
{
  // Get change in time
  current_time_ = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(current_time_ - last_time_);
  elapsed_time_ = ros::Duration(time_span.count());
  last_time_ = current_time_;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_)
  {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycle_time_error_threshold_
                                                               << "s, "
                                                               << "cycle time: " << elapsed_time_ << "s, "
                                                               << "threshold: " << cycle_time_error_threshold_ << "s");
  }

  // Input
  // get the hardware's state
  hardware_interface_->read(ros::Time::now(), elapsed_time_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  // send the new command to hardware
  hardware_interface_->write(ros::Time::now(), elapsed_time_);
}
