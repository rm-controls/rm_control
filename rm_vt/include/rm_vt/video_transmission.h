//
// Created by ch on 24-11-23.
//
#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "rm_vt/common/data.h"
#include "rm_vt/video_transmission_sender.h"

namespace rm_vt
{
class VideoTransmission
{
public:
  explicit VideoTransmission(ros::NodeHandle& nh, Base& base) : base_(base), last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("Video transmission load.");
    custom_controller_cmd_pub_ = nh.advertise<rm_msgs::CustomControllerData>("custom_controller_data", 1);
    vt_keyboard_mouse_pub_ = nh.advertise<rm_msgs::VTKeyboardMouseData>("keyboard_mouse_data", 1);
    vt_receiver_control_pub_ = nh.advertise<rm_msgs::VTReceiverControlData>("receiver_control_data", 1);
    robot_custom_data_pub_ = nh.advertise<rm_msgs::RobotCustomData>("robot_custom_data", 1);
    robot_custom_data_2_pub_ = nh.advertise<rm_msgs::RobotCustomData2>("robot_custom_data_2", 1);
    custom_client_cmd_pub_ = nh.advertise<rm_msgs::CustomClientCmdData>("custom_client_cmd_data", 1);
  }
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher custom_controller_cmd_pub_, vt_keyboard_mouse_pub_, vt_receiver_control_pub_;
  ros::Publisher robot_custom_data_pub_, robot_custom_data_2_pub_, custom_client_cmd_pub_;

  Base& base_;
  std::vector<uint8_t> rx_buffer_;
  int rx_len_{};

private:
  int unpack(uint8_t* rx_data);
  int control_data_unpack(uint8_t* rx_data);
  ros::Time last_get_data_time_;
  const int k_frame_length_ = 309, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 512;
  uint8_t unpack_buffer_[512]{};
};
}  // namespace rm_vt
