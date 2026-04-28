//
// Created by ljyi on 2026/4/6.
//

#pragma once

#include <ros/ros.h>
#include <cstring>
#include "common/data.h"
#include "common/protocol.h"
#include "rm_msgs/VideoPacket.h"

namespace rm_vt
{
class VideoTransmissionSender
{
public:
  explicit VideoTransmissionSender(ros::NodeHandle& nh, Base& base) : base_(base)
  {
    ROS_INFO("Video transmission sender load.");
    deploy_video_stream_sub_ = nh.subscribe("/video_stream", 10, &VideoTransmissionSender::deployVideoStreamCB, this);
  }
  virtual ~VideoTransmissionSender() = default;

private:
  void deployVideoStreamCB(const rm_msgs::VideoPacketConstPtr& msg);
  void sendDeployVideoStream(const uint8_t* data);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len);
  void clearTxBuffer()
  {
    for (int i = 0; i < k_frame_length_; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }

  ros::Subscriber deploy_video_stream_sub_;
  ros::Time deploy_video_stream_last_send_;
  Base& base_;
  uint8_t tx_buffer_[309]{};
  int tx_len_{};

  const int k_frame_length_ = 309, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

}  // namespace rm_vt
