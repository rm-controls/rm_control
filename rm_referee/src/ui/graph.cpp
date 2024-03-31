//
// Created by peter on 2021/7/22.
//
#include "rm_referee/ui/graph.h"
namespace rm_referee
{
Graph::Graph(const XmlRpc::XmlRpcValue& config, Base& base, int id) : base_(base)
{
  config_.graphic_id[0] = static_cast<uint8_t>((id >> 0 & 0xFF));
  config_.graphic_id[1] = static_cast<uint8_t>((id >> 8 & 0xFF));
  config_.graphic_id[2] = static_cast<uint8_t>((id >> 16 & 0xFF));
  if (config.hasMember("type"))
    config_.graphic_type = getType(config["type"]);
  else
  {
    config_.graphic_type = rm_referee::GraphType::STRING;
  }
  if (config_.graphic_type == getType("string") || config_.graphic_type == getType("int_num") ||
      config_.graphic_type == getType("float_num"))
  {
    if (config.hasMember("size"))
      config_.start_angle = static_cast<int>(config["size"]);
  }
  else
  {
    if (config.hasMember("start_angle"))
      config_.start_angle = static_cast<int>(config["start_angle"]);
  }
  if (config.hasMember("start_position"))
  {
    initPosition(config["start_position"], start_positions_);
    if (!start_positions_.empty())
    {
      config_.start_x = start_positions_[0].first;
      config_.start_y = start_positions_[0].second;
    }
  }
  if (config.hasMember("end_position"))
  {
    initPosition(config["end_position"], end_positions_);
    if (!end_positions_.empty())
    {
      config_.end_x = end_positions_[0].first;
      config_.end_y = end_positions_[0].second;
    }
  }
  if (config.hasMember("color"))
    config_.color = getColor(config["color"]);
  else
  {
    config_.color = rm_referee::GraphColor::WHITE;
  }
  if (config.hasMember("end_angle"))
    config_.end_angle = static_cast<int>(config["end_angle"]);
  if (config.hasMember("radius"))
    config_.radius = static_cast<int>(config["radius"]);
  if (config.hasMember("width"))
    config_.width = static_cast<int>(config["width"]);
  if (config.hasMember("title"))
    title_ = static_cast<std::string>(config["title"]);
  if (config.hasMember("content"))
  {
    content_ = static_cast<std::string>(config["content"]);
    if (!title_.empty() || !content_.empty())
      config_.end_angle = static_cast<int>((title_ + content_).size());
  }
  config_.operate_type = rm_referee::GraphOperation::DELETE;
}

void Graph::updatePosition(int index)
{
  if (start_positions_.size() > 1)
  {
    config_.start_x = start_positions_[index].first;
    config_.start_y = start_positions_[index].second;
  }
  if (end_positions_.size() > 1)
  {
    config_.end_x = end_positions_[index].first;
    config_.end_y = end_positions_[index].second;
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
  else if (type == "int_num")
    return rm_referee::GraphType::INT_NUM;
  else if (type == "float_num")
    return rm_referee::GraphType::FLOAT_NUM;
  else if (type == "line")
    return rm_referee::GraphType::LINE;
  else
    return rm_referee::GraphType::LINE;
}

}  // namespace rm_referee
