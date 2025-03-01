//
// Created by chen on 24-11-23.
//
#include "rm_vt/video_tran.h"

namespace rm_vt
{
void VideoTran::read()
{
  if (base_.serial_.available())
  {
    rx_len_ = static_cast<int>(base_.serial_.available());
    base_.serial_.read(rx_buffer_, rx_len_);
  }
  else
    return;
  uint8_t temp_buffer[256] = { 0 };
  int frame_len;
  if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
    base_.video_tran_is_online_ = false;
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = temp_buffer[k_i];
  }
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (unpack_buffer_[k_i] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[k_i]);
      if (frame_len != -1)
        k_i += frame_len;
    }
  }
  clearRxBuffer();
}

int VideoTran::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  rm_vt::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    if (frame_header.data_length > 256)  // temporary and inaccurate value
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length);
      return 0;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case rm_vt::CUSTOM_CONTROLLER_CMD:
        {
          rm_vt::CustomControllerData custom_controller_data;
          std_msgs::Float64MultiArray custom_controller_joint_state;
          memcpy(&custom_controller_data, rx_data + 7, sizeof(rm_vt::CustomControllerData));
          for (int i = 0; i < 5; i++)
          {
            custom_controller_joint_state.data.push_back((1.0 * ((uint16_t)(custom_controller_data.data[2 * i] << 8) |
                                                                 (uint16_t)(custom_controller_data.data[2 * i + 1]))) /
                                                         18000.0 * 3.14);
          }
          custom_controller_cmd_pub_.publish(custom_controller_joint_state);
          break;
        }
        case rm_vt::ROBOT_COMMAND_CMD:
          break;
        default:
          ROS_WARN("Referee command ID %d not found.", cmd_id);
          break;
      }
      base_.video_tran_is_online_ = true;
      last_get_data_time_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}
}  // namespace rm_vt
