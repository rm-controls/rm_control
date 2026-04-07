//
// Created by ljyi on 2026/4/6.
//

#include "rm_vt/video_transmission_sender.h"
#include <cstring>

namespace rm_vt
{
VideoTransmissionBase::VideoTransmissionBase(ros::NodeHandle& nh, Base& base) : base_(base), tx_len_(0)
{
  deploy_video_sub_ = nh.subscribe("/video_stream", 10, &VideoTransmissionBase::videoStreamCB, this);
}

void VideoTransmissionBase::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int data_len)
{
  memset(tx_buffer, 0, k_header_length_ + k_cmd_id_length_ + data_len + k_tail_length_);
  auto* frame_header = reinterpret_cast<FrameHeader*>(tx_buffer);
  frame_header->sof = 0xA5;
  frame_header->data_length = data_len;
  memcpy(tx_buffer + k_header_length_, reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(tx_buffer + k_header_length_ + k_cmd_id_length_, data, data_len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + data_len + k_tail_length_);
}

void VideoTransmissionBase::clearTxBuffer()
{
  memset(tx_buffer_, 0, sizeof(tx_buffer_));
  tx_len_ = 0;
}

void VideoTransmissionBase::sendVideoPacket(const uint8_t* data, size_t data_len)
{
  uint8_t tx_data[sizeof(rm_vt::RobotToCustomData2)] = { 0 };
  auto* video_data = reinterpret_cast<rm_vt::RobotToCustomData2*>(tx_data);
  memcpy(video_data->data, data, data_len);
  int total_len = k_header_length_ + k_cmd_id_length_ + sizeof(rm_vt::RobotToCustomData2) + k_tail_length_;
  for (int i = 0; i < total_len; ++i)
    tx_buffer_[i] = 0;
  pack(tx_buffer_, tx_data, ROBOT_TO_CUSTOM_CLIENT_CMD, sizeof(rm_vt::RobotToCustomData2));
  try
  {
    base_.serial_.write(tx_buffer_, total_len);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void VideoTransmissionBase::videoStreamCB(const rm_msgs::VideoPacketConstPtr& msg)
{
  sendVideoPacket(msg->data.data(), 300);
}

}  // namespace rm_vt
