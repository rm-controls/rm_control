//
// Created by ljyi on 2026/4/6.
//

#pragma once

#include <ros/ros.h>
#include "common/data.h"
#include "common/protocol.h"
#include "rm_msgs/VideoPacket.h"

namespace rm_vt
{
class VideoTransmissionBase
{
public:
  explicit VideoTransmissionBase(ros::NodeHandle& nh, Base& base);
  virtual ~VideoTransmissionBase() = default;

private:
  void videoStreamCB(const rm_msgs::VideoPacketConstPtr& msg);
  void sendVideoPacket(const uint8_t* data, size_t data_len);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int data_len);
  void clearTxBuffer();

  Base& base_;
  ros::Subscriber deploy_video_sub_;
  uint8_t tx_buffer_[309]{};
  int tx_len_{};
  ros::Timer send_serial_data_timer_;

  static const int k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

}  // namespace rm_vt
