//
// Created by ljq on 2021/12/3.
//

#pragma once

#include "rm_referee/referee/ui.h"
#include "rm_referee/common/data.h"

#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>

namespace rm_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(ros::NodeHandle& nh);
  virtual void run();

protected:
  virtual void drawUi(const ros::Time& time)
  {
    data_.referee_.sendUi(time);
  }

  Data data_;
  ros::NodeHandle nh_;
};
}  // namespace rm_referee
