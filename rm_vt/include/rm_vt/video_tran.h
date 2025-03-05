//
// Created by chen on 24-11-23.
//
#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "rm_vt/common/data.h"

namespace rm_vt
{
class VideoTran
{
public:
  explicit VideoTran(ros::NodeHandle& nh) : last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("Video transmission load.");
    custom_controller_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("custom_controller_data", 1);
    base_.initSerial();
  }
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher custom_controller_cmd_pub_;

  Base base_;
  std::vector<uint8_t> rx_buffer_;
  int rx_len_;

private:
  int unpack(uint8_t* rx_data);
  ros::Time last_get_data_time_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};
};
}  // namespace rm_vt
