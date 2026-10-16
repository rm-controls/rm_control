//
// Created by ljyi on 2026/4/6.
//

#include "rm_vt/video_transmission_sender.h"

namespace rm_vt
{
void VideoTransmissionSender::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int data_len)
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = reinterpret_cast<FrameHeader*>(tx_buffer);

  frame_header->sof = 0xA5;
  frame_header->data_length = data_len;
  memcpy(&tx_buffer[k_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, data_len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + data_len + k_tail_length_);
}

void VideoTransmissionSender::sendDeployVideoStream(const uint8_t* data)
{
  uint8_t tx_data[sizeof(rm_vt::RobotToCustomData2)] = { 0 };
  auto video_data = (rm_vt::RobotToCustomData2*)tx_data;

  for (int i = 0; i < 308; i++)
    tx_buffer_[i] = 0;
  for (int i = 0; i < 300; i++)
    video_data->data[i] = data[i];
  pack(tx_buffer_, tx_data, rm_vt::ROBOT_TO_CUSTOM_CLIENT_CMD, sizeof(rm_vt::RobotToCustomData2));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_vt::RobotToCustomData2) + k_tail_length_);
  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void VideoTransmissionSender::deployVideoStreamCB(const rm_msgs::VideoPacketConstPtr& msg)
{
  if (ros::Time::now() - deploy_video_stream_last_send_ <= ros::Duration(0.02))
    return;
  else
  {
    sendDeployVideoStream(msg->data.data());
    deploy_video_stream_last_send_ = ros::Time::now();
  }
}

}  // namespace rm_vt
