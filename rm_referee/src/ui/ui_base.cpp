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
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

void UiBase::erasure()
{
  graph_->setOperation(rm_referee::GraphOperation::DELETE);
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

void GroupUiBase::add()
{
  for (auto graph : graph_vector_)
  {
    graph.second->setOperation(rm_referee::GraphOperation::ADD);
    graph.second->display();
    graph.second->sendUi(ros::Time::now());
  }
}

void GroupUiBase::erasure()
{
  for (auto graph : graph_vector_)
  {
    graph_->setOperation(rm_referee::GraphOperation::DELETE);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }
}

}  // namespace rm_referee
