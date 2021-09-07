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
// Created by qiayuan on 1/3/21.
//

#pragma once

#include <utility>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rm_common/tf_rt_broadcaster.h"

namespace rm_control
{
class RobotStateHandle
{
public:
  RobotStateHandle() = default;
  RobotStateHandle(std::string name, tf2_ros::Buffer* buffer) : name_(std::move(name)), buffer_(buffer)
  {
    if (!buffer)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Tf Buffer data pointer is null.");
  };

  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time)
  {
    return buffer_->lookupTransform(target_frame, source_frame, time);
  }

  bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority,
                    bool is_static = false) const
  {
    return buffer_->setTransform(transform, authority, is_static);
  }

  bool setTransform(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority,
                    bool is_static = false) const
  {
    for (const auto& transform : transforms)
      buffer_->setTransform(transform, authority, is_static);
    return true;
  }

  std::string getName() const
  {
    return name_;
  }

private:
  std::string name_;
  tf2_ros::Buffer* buffer_{};
};

class RobotStateInterface
  : public hardware_interface::HardwareResourceManager<RobotStateHandle, hardware_interface::DontClaimResources>
{
};
}  // namespace rm_control
