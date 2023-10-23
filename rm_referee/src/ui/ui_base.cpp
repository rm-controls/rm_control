//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/ui_base.h"

namespace rm_referee
{
int UiBase::id_(2);
void UiBase::add()
{
  graph_->setOperation(rm_referee::GraphOperation::ADD);
  displayTwice(false);
}

void UiBase::addForQueue(int add_times)
{
  for (int i = 0; i < add_times; i++)
  {
    graph_->setOperation(rm_referee::GraphOperation::ADD);
    graph_queue_->push_back(*graph_);
    last_send_ = ros::Time::now();
  }
}

void UiBase::update()
{
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  display();
}

void UiBase::erasure()
{
  graph_->setOperation(rm_referee::GraphOperation::DELETE);
  displayTwice(false);
}

void GroupUiBase::add()
{
  for (auto graph : graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::ADD);
  for (auto character : character_vector_)
    character.second->setOperation(rm_referee::GraphOperation::ADD);
  displayTwice(false);
}

void GroupUiBase::addForQueue(int add_times)
{
  for (int i = 0; i < add_times; i++)
  {
    for (auto graph : graph_vector_)
    {
      graph.second->setOperation(rm_referee::GraphOperation::ADD);
      graph_queue_->push_back(*graph.second);
      last_send_ = ros::Time::now();
    }
  }
}

void GroupUiBase::update()
{
  for (auto graph : graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
  for (auto character : character_vector_)
    character.second->setOperation(rm_referee::GraphOperation::UPDATE);
  display();
}

void GroupUiBase::erasure()
{
  for (auto graph : graph_vector_)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  for (auto character : character_vector_)
    character.second->setOperation(rm_referee::GraphOperation::DELETE);
  displayTwice(false);
}

void UiBase::display(bool check_repeat)
{
  if (check_repeat)
    if (graph_->isRepeated())
      return;
  graph_->updateLastConfig();
  sendUi(ros::Time::now());
}

void UiBase::displayTwice(bool check_repeat)
{
  if (check_repeat)
    if (graph_->isRepeated())
      return;
  graph_->updateLastConfig();
  for (int i = 0; i < 2; ++i)
    sendUi(ros::Time::now());
}

void UiBase::display(const ros::Time& time)
{
  if (time - last_send_ < delay_)
    return;
  display();
}

void UiBase::display(const ros::Time& time, bool state, bool once)
{
  if (once)
  {
    if (state)
      graph_->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph_->setOperation(rm_referee::GraphOperation::DELETE);
  }
  else if (state && time - last_send_ > delay_)
  {
    // todo: has problem in this case
    ROS_INFO("%f  %.3f", last_send_.toSec(), delay_.toSec());
    graph_->setOperation(graph_->getOperation() == rm_referee::GraphOperation::ADD ?
                             rm_referee::GraphOperation::DELETE :
                             rm_referee::GraphOperation::ADD);
  }
  displayTwice();
}

void UiBase::sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data)
{
  uint8_t tx_data[sizeof(InteractiveData)] = { 0 };
  auto student_interactive_data = (InteractiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data.data_cmd_id = data_cmd_id;
  student_interactive_data->header_data.sender_id = base_.robot_id_;
  student_interactive_data->header_data.receiver_id = receiver_id;
  student_interactive_data->data = data;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(InteractiveData) + k_tail_length_);

  sendSerial(ros::Time::now(), sizeof(InteractiveData));
}

void UiBase::sendCurrentSentryData(const rm_msgs::CurrentSentryPosDataConstPtr& data)
{
  int data_len;
  uint8_t tx_data[sizeof(CurrentSentryPosData)] = { 0 };
  auto current_sentry_pos_data = (CurrentSentryPosData*)tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::CurrentSentryPosData));

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;

  current_sentry_pos_data->header_data.data_cmd_id = DataCmdId::CURRENT_SENTRY_POSITION_CMD;
  current_sentry_pos_data->header_data.sender_id = base_.robot_id_;
  current_sentry_pos_data->header_data.receiver_id = base_.robot_id_ < 100 ? RobotId::RED_SENTRY : RobotId::BLUE_SENTRY;
  current_sentry_pos_data->position_x = data->x;
  current_sentry_pos_data->position_y = data->y;
  current_sentry_pos_data->position_z = data->z;
  current_sentry_pos_data->position_yaw = data->yaw;

  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
  sendSerial(ros::Time::now(), data_len);
}

void UiBase::sendUi(const ros::Time& time)
{
  if (base_.robot_id_ == 0 || base_.client_id_ == 0)
    return;

  std::string characters = graph_->getCharacters();
  if (!characters.empty())
    sendCharacter(time, graph_);
  else
    sendSingleGraph(time, graph_);
}

void UiBase::sendMapSentryData(const rm_msgs::MapSentryDataConstPtr& data)
{
  uint8_t tx_data[sizeof(rm_referee::MapSentryData)] = { 0 };
  auto map_sentry_data = (rm_referee::MapSentryData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  map_sentry_data->intention = data->intention;
  map_sentry_data->start_position_x = data->start_position_x;
  map_sentry_data->start_position_y = data->start_position_y;
  for (int i = 0; i < 49; i++)
  {
    map_sentry_data->delta_x[i] = data->delta_x[i];
    map_sentry_data->delta_y[i] = data->delta_y[i];
  }
  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::MAP_SENTRY_CMD, sizeof(rm_referee::MapSentryData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_referee::MapSentryData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
  }

  clearTxBuffer();
}

void UiBase::sendRadarInteractiveData(rm_referee::ClientMapReceiveData& data)
{
  uint8_t tx_data[sizeof(rm_referee::ClientMapReceiveData)] = { 0 };
  auto radar_interactive_data = (rm_referee::ClientMapReceiveData*)tx_data;

  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  radar_interactive_data->target_robot_ID = data.target_robot_ID;
  radar_interactive_data->target_position_x = data.target_position_x;
  radar_interactive_data->target_position_y = data.target_position_y;
  pack(tx_buffer_, tx_data, rm_referee::RefereeCmdId::CLIENT_MAP_CMD, sizeof(rm_referee::ClientMapReceiveData));
  tx_len_ =
      k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm_referee::ClientMapReceiveData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
  }

  clearTxBuffer();
}

void UiBase::sendCharacter(const ros::Time& time, rm_referee::Graph* graph)
{
  int data_len;
  std::string characters = graph->getCharacters();
  rm_referee::CharacterData tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::CharacterData));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = base_.client_id_;
  tx_data.config = graph->getConfig();

  for (int i = 0; i < 30; i++)
  {
    if (i < static_cast<int>(characters.size()))
      tx_data.content[i] = characters[i];
    else
      tx_data.content[i] = ' ';
  }
  tx_data.header.data_cmd_id = rm_referee::DataCmdId::CLIENT_CHARACTER_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(time, data_len);
}

void UiBase::sendSingleGraph(const ros::Time& time, Graph* graph)
{
  int data_len;
  rm_referee::SingleGraphData tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::SingleGraphData));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = base_.client_id_;
  tx_data.config = graph->getConfig();

  tx_data.header.data_cmd_id = rm_referee::DataCmdId::CLIENT_GRAPH_SINGLE_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(time, data_len);
}

void GroupUiBase::display(bool check_repeat)
{
  if (check_repeat)
  {
    bool is_repeat = true;
    for (auto it : graph_vector_)
      if (!it.second->isRepeated())
        is_repeat = false;
    for (auto it : character_vector_)
      if (!it.second->isRepeated())
        is_repeat = false;
    if (is_repeat)
      return;
  }

  for (auto it : graph_vector_)
    it.second->updateLastConfig();
  for (auto it : character_vector_)
    it.second->updateLastConfig();
  sendUi(ros::Time::now());
}

void GroupUiBase::displayTwice(bool check_repeat)
{
  if (check_repeat)
  {
    bool is_repeat = true;
    for (auto it : graph_vector_)
      if (!it.second->isRepeated())
        is_repeat = false;
    for (auto it : character_vector_)
      if (!it.second->isRepeated())
        is_repeat = false;
    if (is_repeat)
      return;
  }

  for (auto it : graph_vector_)
    it.second->updateLastConfig();
  for (auto it : character_vector_)
    it.second->updateLastConfig();
  for (int i = 0; i < 2; i++)
    sendUi(ros::Time::now());
}

void GroupUiBase::display(const ros::Time& time)
{
  if (time - last_send_ < delay_)
    return;
  display();
}

void GroupUiBase::sendUi(const ros::Time& time)
{
  if (base_.robot_id_ == 0 || base_.client_id_ == 0)
    return;

  for (auto it : character_vector_)
    sendCharacter(time, it.second);
  for (auto it : graph_vector_)
    sendSingleGraph(time, it.second);
}

void GroupUiBase::sendDoubleGraph(const ros::Time& time, Graph* graph0, Graph* graph1)
{
  int data_len;
  rm_referee::DoubleGraphData tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::DoubleGraphData));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = base_.client_id_;
  tx_data.config[0] = graph0->getConfig();
  tx_data.config[1] = graph1->getConfig();

  tx_data.header.data_cmd_id = rm_referee::DataCmdId::CLIENT_GRAPH_DOUBLE_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(time, data_len);
}

void GroupUiBase::sendFiveGraph(const ros::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3,
                                Graph* graph4)
{
  int data_len;
  rm_referee::FiveGraphData tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::FiveGraphData));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = base_.client_id_;
  tx_data.config[0] = graph0->getConfig();
  tx_data.config[1] = graph1->getConfig();
  tx_data.config[2] = graph2->getConfig();
  tx_data.config[3] = graph3->getConfig();
  tx_data.config[4] = graph4->getConfig();

  tx_data.header.data_cmd_id = rm_referee::DataCmdId::CLIENT_GRAPH_FIVE_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(time, data_len);
}

void GroupUiBase::sendSevenGraph(const ros::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3,
                                 Graph* graph4, Graph* graph5, Graph* graph6)
{
  int data_len;
  rm_referee::SevenGraphData tx_data;
  data_len = static_cast<int>(sizeof(rm_referee::SevenGraphData));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = base_.client_id_;
  tx_data.config[0] = graph0->getConfig();
  tx_data.config[1] = graph1->getConfig();
  tx_data.config[3] = graph2->getConfig();
  tx_data.config[4] = graph3->getConfig();
  tx_data.config[5] = graph4->getConfig();
  tx_data.config[6] = graph5->getConfig();

  tx_data.header.data_cmd_id = rm_referee::DataCmdId::CLIENT_GRAPH_SEVEN_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(time, data_len);
}

void UiBase::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = reinterpret_cast<FrameHeader*>(tx_buffer);

  frame_header->sof = 0xA5;
  frame_header->data_length = len;
  memcpy(&tx_buffer[k_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + len + k_tail_length_);
}

void UiBase::sendSerial(const ros::Time& time, int data_len)
{
  tx_len_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_ + data_len;
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

void UiBase::clearTxBuffer()
{
  for (int i = 0; i < 128; i++)
    tx_buffer_[i] = 0;
  tx_len_ = 0;
}

}  // namespace rm_referee
