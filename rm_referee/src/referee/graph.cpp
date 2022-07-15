//
// Created by peter on 2021/7/22.
//
#include "rm_referee/referee/graph.h"
namespace rm_referee
{
Graph::Graph(const XmlRpc::XmlRpcValue& config, Referee& referee, int id) : referee_(referee)
{
  config_.graphic_id_[0] = (uint8_t)(id >> 0 & 0xFF);
  config_.graphic_id_[1] = (uint8_t)(id >> 8 & 0xFF);
  config_.graphic_id_[2] = (uint8_t)(id >> 16 & 0xFF);
  if (config.hasMember("type"))
    config_.graphic_type_ = getType(config["type"]);
  else
    config_.graphic_type_ = rm_common::GraphType::STRING;
  if (config_.graphic_type_ == getType("string"))
  {
    if (config.hasMember("size"))
      config_.start_angle_ = (int)config["size"];
  }
  else
  {
    if (config.hasMember("start_angle"))
      config_.start_angle_ = (int)config["start_angle"];
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
    config_.color_ = rm_common::GraphColor::WHITE;
  }
  if (config.hasMember("end_angle"))
    config_.end_angle_ = (int)config["end_angle"];
  if (config.hasMember("radius"))
    config_.radius_ = (int)config["radius"];
  if (config.hasMember("width"))
    config_.width_ = (int)config["width"];
  if (config.hasMember("delay"))
    delay_ = ros::Duration((double)config["delay"]);
  if (config.hasMember("title"))
    title_ = (std::string)config["title"];
  if (config.hasMember("content"))
    content_ = (std::string)config["content"];
  config_.operate_type_ = rm_common::GraphOperation::DELETE;
  last_config_ = config_;
  last_title_ = title_;
  last_content_ = content_;
}

void Graph::display(bool priority_flag)
{
  if (config_ == last_config_ && title_ == last_title_ && content_ == last_content_)
    return;
  if (!title_.empty() && !content_.empty())
    config_.end_angle_ = (int)(title_ + content_).size();
  referee_.addUi(config_, title_ + content_, priority_flag);
  last_content_ = content_;
  last_title_ = title_;
  last_config_ = config_;
}

void Graph::displayTwice(bool priority_flag)
{
  if (config_ == last_config_ && title_ == last_title_ && content_ == last_content_)
    return;
  if (!title_.empty() && !content_.empty())
    config_.end_angle_ = (int)(title_ + content_).size();
  for (int i = 0; i < 2; ++i)
    referee_.addUi(config_, title_ + content_, priority_flag);
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
      config_.operate_type_ = rm_common::GraphOperation::ADD;
    }
    if (time - last_time_ > delay_)
      config_.operate_type_ = rm_common::GraphOperation::DELETE;
  }
  else if (state && time - last_time_ > delay_)
  {
    config_.operate_type_ = config_.operate_type_ == rm_common::GraphOperation::ADD ?
                                rm_common::GraphOperation::DELETE :
                                rm_common::GraphOperation::ADD;
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
    positions.push_back(std::pair<int, int>((int)value[0], (int)value[1]));
  }
}

rm_common::GraphColor Graph::getColor(const std::string& color)
{
  if (color == "main_color")
    return rm_common::GraphColor::MAIN_COLOR;
  else if (color == "yellow")
    return rm_common::GraphColor::YELLOW;
  else if (color == "green")
    return rm_common::GraphColor::GREEN;
  else if (color == "orange")
    return rm_common::GraphColor::ORANGE;
  else if (color == "purple")
    return rm_common::GraphColor::PURPLE;
  else if (color == "pink")
    return rm_common::GraphColor::PINK;
  else if (color == "cyan")
    return rm_common::GraphColor::CYAN;
  else if (color == "black")
    return rm_common::GraphColor::BLACK;
  else
    return rm_common::GraphColor::WHITE;
}

rm_common::GraphType Graph::getType(const std::string& type)
{
  if (type == "rectangle")
    return rm_common::GraphType::RECTANGLE;
  else if (type == "circle")
    return rm_common::GraphType::CIRCLE;
  else if (type == "ellipse")
    return rm_common::GraphType::ELLIPSE;
  else if (type == "arc")
    return rm_common::GraphType::ARC;
  else if (type == "string")
    return rm_common::GraphType::STRING;
  else
    return rm_common::GraphType::LINE;
}

}  // namespace rm_referee
