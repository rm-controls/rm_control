//
// Created by ch on 24-11-23.
//
#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "rm_vt/common/data.h"

namespace rm_vt
{
class VideoTransmission
{
public:
  static uint16_t keyboardMaskFromLegacyFrame(const rm_vt::KeyboardMouseData& data);
  static uint16_t keyCodeToMask(uint8_t key_code);
  static uint16_t keyboardMaskFromKeyCodes(uint16_t key_value);

  explicit VideoTransmission(ros::NodeHandle& nh) : last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("Video transmission load.");
    custom_controller_cmd_pub_ = nh.advertise<rm_msgs::CustomControllerData>("custom_controller_data", 1);
    vt_keyboard_mouse_pub_ = nh.advertise<rm_msgs::VTKeyboardMouseData>("keyboard_mouse_data", 1);
    vt_receiver_control_pub_ = nh.advertise<rm_msgs::VTReceiverControlData>("receiver_control_data", 1);
    robot_to_custom_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("robot_to_custom_data", 1);
    robot_to_custom2_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("robot_to_custom_data2", 1);
    custom_to_robot_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("custom_to_robot_data", 1);
    base_.initSerial();
  }
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher custom_controller_cmd_pub_, vt_keyboard_mouse_pub_, vt_receiver_control_pub_;
  ros::Publisher robot_to_custom_pub_, robot_to_custom2_pub_, custom_to_robot_pub_;

  Base base_;
  std::vector<uint8_t> rx_buffer_;
  int rx_len_;

private:
  int unpack(uint8_t* rx_data, int rx_data_len);
  int control_data_unpack(uint8_t* rx_data, int rx_data_len);
  void publishKeyboardMouseData(uint16_t keyboard_value, int16_t mouse_x, int16_t mouse_y, int16_t mouse_z,
                                bool left_button_down, bool right_button_down);
  uint16_t updateKeyboardValueStateFromKeyCodes(uint16_t key_value);
  ros::Time last_get_data_time_;
  static constexpr int k_frame_length_ = 128;
  static constexpr int k_header_length_ = 5;
  static constexpr int k_cmd_id_length_ = 2;
  static constexpr int k_tail_length_ = 2;
  static constexpr int k_unpack_buffer_length_ = 512;
  uint8_t unpack_buffer_[k_unpack_buffer_length_]{};
  uint16_t keyboard_value_state_{ 0 };
};
}  // namespace rm_vt
