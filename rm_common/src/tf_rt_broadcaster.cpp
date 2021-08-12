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
#include "rm_common/tf_rt_broadcaster.h"

#include <vector>
#include <tf2_msgs/TFMessage.h>

namespace rm_common
{
void TfRtBroadcaster::init(ros::NodeHandle& root_nh)
{
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
}

void TfRtBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void TfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
  tf2_msgs::TFMessage message;
  for (const auto& transform : transforms)
  {
    message.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock())
  {
    realtime_pub_->msg_ = message;
    realtime_pub_->unlockAndPublish();
  }
}

void StaticTfRtBroadcaster::init(ros::NodeHandle& root_nh)
{
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf_static", 100, true));
}

void StaticTfRtBroadcaster::sendTransform(const geometry_msgs::TransformStamped& transform)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void StaticTfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
  for (const auto& transform : transforms)
  {
    bool match_found = false;
    for (auto& it_msg : net_message_.transforms)
    {
      if (transform.child_frame_id == it_msg.child_frame_id)
      {
        it_msg = transform;
        match_found = true;
        break;
      }
    }
    if (!match_found)
      net_message_.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock())
  {
    realtime_pub_->msg_ = net_message_;
    realtime_pub_->unlockAndPublish();
  }
}

}  // namespace rm_common
