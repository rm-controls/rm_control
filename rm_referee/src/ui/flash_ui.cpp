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
    if (state)
      graph_->setOperation(rm_referee::GraphOperation::ADD);
    else
      graph_->setOperation(rm_referee::GraphOperation::DELETE);
  }
  else if (state && time - last_send_ > delay_)
  {
    ROS_INFO("%f  %.3f", last_send_.toSec(), delay_.toSec());
    graph_->setOperation(graph_->getOperation() == rm_referee::GraphOperation::ADD ?
                             rm_referee::GraphOperation::DELETE :
                             rm_referee::GraphOperation::ADD);
  }
  if (graph_->isRepeated())
    return;
  graph_->updateLastConfig();
  UiBase::updateForQueue();
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
}  // namespace rm_referee
