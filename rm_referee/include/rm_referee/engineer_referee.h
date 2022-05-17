//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/robot_referee.h"

#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/EngineerAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>

namespace rm_referee
{
class EngineerReferee : public RobotReferee
{
public:
  explicit EngineerReferee(ros::NodeHandle& nh);
  void run() override;

private:
  void drawUi(const ros::Time& time) override;
  void drawProcess(const ros::Time& time);
  bool symbol;
};

}  // namespace rm_referee