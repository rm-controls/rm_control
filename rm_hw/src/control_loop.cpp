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

namespace rm_hw
{
RmRobotHWLoop::RmRobotHWLoop(ros::NodeHandle& nh, std::shared_ptr<RmRobotHW> hardware_interface)
  : nh_(nh), hardware_interface_(std::move(hardware_interface))
{
  // Load ros params
  int error = 0, thread_priority;
  ros::NodeHandle nh_p("~");
  error += !nh_p.getParam("loop_frequency", loop_hz_);
  error += !nh_p.getParam("cycle_time_error_threshold", cycle_time_error_threshold_);
  error += !nh_p.getParam("thread_priority", thread_priority);
  if (error > 0)
  {
    char error_message[] = "could not retrieve one of the required parameters\n\trm_hw/loop_hz or "
                           "rm_hw/cycle_time_error_threshold or "
                           "rm_hw/thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Initialise the hardware interface:
  // 1. retrieve configuration from rosparam
  // 2. initialise the hardware and interface it with ros_control
  hardware_interface_->setCanBusThreadPriority(thread_priority);
  hardware_interface_->init(nh, nh_p);
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Get current time for use with first update
  last_time_ = clock::now();

  // Setup loop thread
  loop_thread_ = std::thread([&]() {
    while (loop_running_)
    {
      if (loop_running_)
        update();
    }
  });
  sched_param sched{ .sched_priority = thread_priority };
  if (pthread_setschedparam(loop_thread_.native_handle(), SCHED_FIFO, &sched) != 0)
    ROS_WARN("Failed to set threads priority (one possible reason could be that the user and the group permissions "
             "are not set properly.).\n");
}

void RmRobotHWLoop::update()
{
  const auto current_time = clock::now();
  // Compute desired duration rounded to clock decimation
  const duration<double> desired_duration(1.0 / loop_hz_);
  // Get change in time
  duration<double> time_span = duration_cast<duration<double>>(current_time - last_time_);
  elapsed_time_ = ros::Duration(time_span.count());
  last_time_ = current_time;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsed_time_ - ros::Duration(desired_duration.count())).toSec();
  if (cycle_time_error > cycle_time_error_threshold_)
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycle_time_error_threshold_
                                                               << "s, "
                                                               << "cycle time: " << elapsed_time_ << "s, "
                                                               << "threshold: " << cycle_time_error_threshold_ << "s");
  // Input
  // get the hardware's state
  hardware_interface_->read(ros::Time::now(), elapsed_time_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  // send the new command to hardware
  hardware_interface_->write(ros::Time::now(), elapsed_time_);

  // Sleep
  const auto sleep_till = current_time + duration_cast<clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);
}

RmRobotHWLoop::~RmRobotHWLoop()
{
  loop_running_ = false;
  if (loop_thread_.joinable())
    loop_thread_.join();
}
}  // namespace rm_hw
