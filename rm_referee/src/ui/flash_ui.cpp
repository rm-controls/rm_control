//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/flash_ui.h"

namespace rm_referee
{
void FlashUi::updateFlashUiForQueue(const ros::Time& time, bool state, bool once)
{
  if (once)
  {
    //    ROS_INFO("test");
    if (state)
      graph_->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph_->setOperation(rm_referee::GraphOperation::DELETE);
  }
  else if (time - last_send_ > delay_)
  {
    //    ROS_INFO("%f  %.3f", last_send_.toSec(), delay_.toSec());
    if (state)
      graph_->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph_->setOperation(rm_referee::GraphOperation::DELETE);
    //    graph_->setOperation(graph_->getOperation() == rm_referee::GraphOperation::ADD ?
    //                             rm_referee::GraphOperation::DELETE :
    //                             rm_referee::GraphOperation::ADD);
    //    ROS_INFO("delay");
  }
  if (graph_->isRepeated())
    return;
  graph_->updateLastConfig();
  last_send_ = time;
  UiBase::updateForQueue();
}

void FlashGroupUi::updateFlashUiForQueue(const ros::Time& time, bool state, bool once, Graph* graph)
{
  if (once)
  {
    if (state)
      graph->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph->setOperation(rm_referee::GraphOperation::DELETE);
  }
  else if (time - last_send_ > delay_)
  {
    ROS_INFO("%f  %.3f", last_send_.toSec(), delay_.toSec());
    if (state)
      graph->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph->setOperation(rm_referee::GraphOperation::DELETE);
  }
  if (graph->isRepeated())
    return;
  graph->updateLastConfig();
  last_send_ = time;
  if (graph->isString())
    character_queue_->push_back(*graph);
  else
    graph_queue_->push_back(*graph);
}

void EngineerActionFlashUi::updateEngineerUiCmdData(const rm_msgs::EngineerUi::ConstPtr data,
                                                    const ros::Time& last_get_data_time)
{
  symbol_ = data->symbol;
  display(last_get_data_time);
}

void EngineerActionFlashUi::display(const ros::Time& time)
{
  for (auto graph : graph_vector_)
  {
    bool state = false;
    if (std::to_string(static_cast<int>(symbol_)) == graph.first)
      state = true;
    FlashGroupUi::updateFlashUiForQueue(time, state, true, graph.second);
  }
}

void CoverFlashUi::display(const ros::Time& time)
{
  if (!cover_state_)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  FlashUi::updateFlashUiForQueue(time, cover_state_, true);
}

void CoverFlashUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data,
                                       const ros::Time& last_get_data_time)
{
  cover_state_ = data->cover_state;
  display(last_get_data_time);
}

void SpinFlashUi::display(const ros::Time& time)
{
  if (chassis_mode_ != rm_msgs::ChassisCmd::RAW)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  FlashUi::updateFlashUiForQueue(time, chassis_mode_ != rm_msgs::ChassisCmd::RAW, true);
}

void SpinFlashUi::updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data, const ros::Time& last_get_data_time)
{
  chassis_mode_ = data->mode;
  display(last_get_data_time);
}

void HeroHitFlashUi::updateHittingConfig(const rm_msgs::GameRobotHp& msg)
{
  if (base_.robot_id_ > 100)
  {
    hitted_ =
        (last_hp_msg_.red_outpost_hp - msg.red_outpost_hp > 190 || last_hp_msg_.red_base_hp - msg.red_base_hp > 190);
  }
  else
  {
    hitted_ = (last_hp_msg_.blue_outpost_hp - msg.blue_outpost_hp > 190 ||
               last_hp_msg_.blue_base_hp - msg.blue_base_hp > 190);
  }
  last_hp_msg_ = msg;
  display(ros::Time::now());
}

void HeroHitFlashUi::display(const ros::Time& time)
{
  if (hitted_)
    FlashUi::updateFlashUiForQueue(time, true, true);
  else
    FlashUi::updateFlashUiForQueue(time, false, false);
}

void ExceedBulletSpeedFlashUi::display(const ros::Time& time)
{
  if (shoot_data_.bullet_speed <= 30)
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
  FlashUi::updateFlashUiForQueue(time, shoot_data_.bullet_speed > 30, true);
}

void ExceedBulletSpeedFlashUi::updateShootData(const rm_msgs::ShootData& msg)
{
  shoot_data_ = msg;
}

}  // namespace rm_referee
// namespace rm_referee
