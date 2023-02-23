//
// Created by peter on 2021/7/22.
//
#include "rm_referee/ui/graph.h"
namespace rm_referee
{
Graph::Graph(const XmlRpc::XmlRpcValue& config, Base& base, int id) : base_(base), last_send_(ros::Time::now())
{
  config_.graphic_id_[0] = static_cast<uint8_t>((id >> 0 & 0xFF));
  config_.graphic_id_[1] = static_cast<uint8_t>((id >> 8 & 0xFF));
  config_.graphic_id_[2] = static_cast<uint8_t>((id >> 16 & 0xFF));
  if (config.hasMember("type"))
    config_.graphic_type_ = getType(config["type"]);
  else
    config_.graphic_type_ = rm_referee::GraphType::STRING;
  if (config_.graphic_type_ == getType("string"))
  {
    if (config.hasMember("size"))
      config_.start_angle_ = static_cast<int>(config["size"]);
  }
  else
  {
    if (config.hasMember("start_angle"))
      config_.start_angle_ = static_cast<int>(config["start_angle"]);
  }
  if (config.hasMember("start_position"))
  {
    initPosition(config["start_position"], start_positions_);
    if (!start_positions_.empty())
    {
      config_.start_x_ = start_positions_[0].first;
      config_.start_y_ = start_positions_[0].second;
    }
  }
  if (config.hasMember("end_position"))
  {
    initPosition(config["end_position"], end_positions_);
    if (!end_positions_.empty())
    {
      config_.end_x_ = end_positions_[0].first;
      config_.end_y_ = end_positions_[0].second;
    }
  }
  if (config.hasMember("color"))
    config_.color_ = getColor(config["color"]);
  else
  {
    config_.color_ = rm_referee::GraphColor::WHITE;
  }
  if (config.hasMember("end_angle"))
    config_.end_angle_ = static_cast<int>(config["end_angle"]);
  if (config.hasMember("radius"))
    config_.radius_ = static_cast<int>(config["radius"]);
  if (config.hasMember("width"))
    config_.width_ = static_cast<int>(config["width"]);
  if (config.hasMember("delay"))
    delay_ = ros::Duration(static_cast<double>(config["delay"]));
  if (config.hasMember("title"))
    title_ = static_cast<std::string>(config["title"]);
  if (config.hasMember("content"))
    content_ = static_cast<std::string>(config["content"]);
  config_.operate_type_ = rm_referee::GraphOperation::DELETE;
  last_config_ = config_;
  last_title_ = title_;
  last_content_ = content_;
}

Graph::Graph(Base& base) : base_(base)
{
}

void Graph::display(bool priority_flag)
{
  if (config_ == last_config_ && title_ == last_title_ && content_ == last_content_)
    return;
  if (!title_.empty() && !content_.empty())
    config_.end_angle_ = static_cast<int>((title_ + content_).size());
  addUi(config_, title_ + content_, priority_flag);
  last_content_ = content_;
  last_title_ = title_;
  last_config_ = config_;
}

void Graph::displayTwice(bool priority_flag)
{
  if (config_ == last_config_ && title_ == last_title_ && content_ == last_content_)
    return;
  if (!title_.empty() && !content_.empty())
    config_.end_angle_ = static_cast<int>((title_ + content_).size());
  for (int i = 0; i < 2; ++i)
    addUi(config_, title_ + content_, priority_flag);
  last_content_ = content_;
  last_title_ = title_;
  last_config_ = config_;
}

void Graph::display(const ros::Time& time)
{
  if (time - last_time_ < delay_)
    return;
  display();
  last_time_ = time;
}

void Graph::display(const ros::Time& time, bool state, bool once)
{
  if (once)
  {
    if (state)
    {
      last_time_ = time;
      config_.operate_type_ = rm_referee::GraphOperation::ADD;
    }
    if (time - last_time_ > delay_)
      config_.operate_type_ = rm_referee::GraphOperation::DELETE;
  }
  else if (state && time - last_time_ > delay_)
  {
    config_.operate_type_ = config_.operate_type_ == rm_referee::GraphOperation::ADD ?
                                rm_referee::GraphOperation::DELETE :
                                rm_referee::GraphOperation::ADD;
    last_time_ = time;
  }
  display(true);
}

void Graph::updatePosition(int index)
{
  if (start_positions_.size() > 1)
  {
    config_.start_x_ = start_positions_[index].first;
    config_.start_y_ = start_positions_[index].second;
  }
  if (end_positions_.size() > 1)
  {
    config_.end_x_ = end_positions_[index].first;
    config_.end_y_ = end_positions_[index].second;
  }
}

void Graph::initPosition(XmlRpc::XmlRpcValue value, std::vector<std::pair<int, int>>& positions)
{
  if (value.size() != 0 && value[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < value.size(); ++i)
      initPosition(value[i], positions);
  }
  else
  {
    positions.push_back(std::pair<int, int>(static_cast<int>(value[0]), static_cast<int>(value[1])));
  }
}

rm_referee::GraphColor Graph::getColor(const std::string& color)
{
  if (color == "main_color")
    return rm_referee::GraphColor::MAIN_COLOR;
  else if (color == "yellow")
    return rm_referee::GraphColor::YELLOW;
  else if (color == "green")
    return rm_referee::GraphColor::GREEN;
  else if (color == "orange")
    return rm_referee::GraphColor::ORANGE;
  else if (color == "purple")
    return rm_referee::GraphColor::PURPLE;
  else if (color == "pink")
    return rm_referee::GraphColor::PINK;
  else if (color == "cyan")
    return rm_referee::GraphColor::CYAN;
  else if (color == "black")
    return rm_referee::GraphColor::BLACK;
  else
    return rm_referee::GraphColor::WHITE;
}

rm_referee::GraphType Graph::getType(const std::string& type)
{
  if (type == "rectangle")
    return rm_referee::GraphType::RECTANGLE;
  else if (type == "circle")
    return rm_referee::GraphType::CIRCLE;
  else if (type == "ellipse")
    return rm_referee::GraphType::ELLIPSE;
  else if (type == "arc")
    return rm_referee::GraphType::ARC;
  else if (type == "string")
    return rm_referee::GraphType::STRING;
  else
    return rm_referee::GraphType::LINE;
}

void Graph::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = reinterpret_cast<rm_referee::FrameHeader*>(tx_buffer);

  frame_header->sof_ = 0xA5;
  frame_header->data_length_ = len;
  memcpy(&tx_buffer[k_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + len + k_tail_length_);
}

void Graph::sendInteractiveData(int data_cmd_id, int receiver_id, uint8_t data)
{
  uint8_t tx_data[sizeof(rm_referee::InteractiveData)] = { 0 };
  auto student_interactive_data = (rm_referee::InteractiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data_.data_cmd_id_ = data_cmd_id;
  student_interactive_data->header_data_.sender_id_ = base_.robot_id_;
  student_interactive_data->header_data_.receiver_id_ = receiver_id;
  student_interactive_data->data_ = data;
  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(rm_referee::InteractiveData));
  tx_len_ =
      k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_referee::InteractiveData) + k_tail_length_);
}

void Graph::addUi(const rm_referee::GraphConfig& config, const std::string& content, bool priority_flag)
{
  for (int i = 0; i < static_cast<int>(ui_queue_.size() - 20); i++)
    ui_queue_.erase(ui_queue_.begin());
  if (priority_flag)
    ui_queue_.push_back(std::pair<rm_referee::GraphConfig, std::string>(config, content));
  else
    ui_queue_.insert(ui_queue_.begin(), std::pair<rm_referee::GraphConfig, std::string>(config, content));
}

void Graph::sendUi(const ros::Time& time)
{
  if (ui_queue_.empty() || time - last_send_ < ros::Duration(0.05))
    return;
  rm_referee::GraphData tx_data;
  int data_len = static_cast<int>(sizeof(rm_referee::GraphData));
  if (base_.robot_id_ == 0 || base_.client_id_ == 0)
    return;
  tx_data.header_.sender_id_ = base_.robot_id_;
  tx_data.header_.receiver_id_ = base_.client_id_;
  tx_data.config_ = ui_queue_.back().first;
  if (ui_queue_.back().second.empty())
  {
    tx_data.header_.data_cmd_id_ = rm_referee::DataCmdId::CLIENT_GRAPH_SINGLE_CMD;
    data_len -= 30;
  }
  else
  {
    tx_data.header_.data_cmd_id_ = rm_referee::DataCmdId::CLIENT_CHARACTER_CMD;
    for (int i = 0; i < 30; i++)
    {
      if (i < static_cast<int>(ui_queue_.back().second.size()))
        tx_data.content_[i] = ui_queue_.back().second[i];
      else
        tx_data.content_[i] = ' ';
    }
  }
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  tx_len_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_ + data_len;
  ui_queue_.pop_back();
  last_send_ = time;

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
  }

  clearTxBuffer();
}

}  // namespace rm_referee
