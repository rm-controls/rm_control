//
// Created by peter on 2021/7/19.
//

#pragma once

#include "rm_referee/common/data.h"
#include <rm_common/ros_utilities.h>

namespace rm_referee
{
class Graph
{
public:
  explicit Graph(const XmlRpc::XmlRpcValue& config, Base& base, int id);
  explicit Graph(Base& base);
  void addUi(const rm_referee::GraphConfig& config, const std::string& content, bool priority_flag = false);
  void sendUi(const ros::Time& time);
  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void clearTxBuffer()
  {
    for (int i = 0; i < 128; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }

  void display(bool priority_flag = false);
  void displayTwice(bool priority_flag = false);
  void display(const ros::Time& time);
  void display(const ros::Time& time, bool state, bool once = false);
  void updatePosition(int index);
  void setOperation(const rm_referee::GraphOperation& operation)
  {
    config_.operate_type_ = operation;
  }
  void setColor(const rm_referee::GraphColor& color)
  {
    config_.color_ = color;
  }
  void setContent(const std::string& content)
  {
    content_ = content;
  }
  void setEndX(int end_x)
  {
    config_.end_x_ = end_x;
  }
  void setEndY(int end_y)
  {
    config_.end_y_ = end_y;
  }
  void setStartX(int start_x)
  {
    config_.start_x_ = start_x;
  }
  void setStartY(int start_y)
  {
    config_.start_y_ = start_y;
  }

  uint8_t tx_buffer_[128];
  int tx_len_;

private:
  void initPosition(XmlRpc::XmlRpcValue value, std::vector<std::pair<int, int>>& positions);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const;
  rm_referee::GraphColor getColor(const std::string& color);
  rm_referee::GraphType getType(const std::string& type);

  Base& base_;
  ros::Time last_time_ = ros::Time::now();
  ros::Duration delay_ = ros::Duration(0.);
  std::string title_{}, content_{}, last_title_{}, last_content_{};
  std::vector<std::pair<int, int>> start_positions_{}, end_positions_{};
  rm_referee::GraphConfig config_{}, last_config_{};

  ros::Time last_send_;
  std::vector<std::pair<rm_referee::GraphConfig, std::string>> ui_queue_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

}  // namespace rm_referee
