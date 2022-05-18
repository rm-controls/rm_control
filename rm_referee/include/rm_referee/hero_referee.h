//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/robot_referee.h"
#include <rm_msgs/StatusChange.h>

namespace rm_referee
{
class HeroReferee : public RobotReferee
{
public:
  explicit HeroReferee(ros::NodeHandle& nh) : RobotReferee(nh){};
  void run() override;

protected:
  void drawUi(const ros::Time& time) override;
};
}  // namespace rm_referee