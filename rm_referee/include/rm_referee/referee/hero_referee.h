//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/referee/robot_referee.h"
#include <rm_msgs/StatusChange.h>
#include <rm_common/decision/calibration_queue.h>

namespace rm_referee
{
class HeroReferee : public RobotReferee
{
public:
  explicit HeroReferee(ros::NodeHandle& nh, Base& base);
  void capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, const ros::Time& last_get_) override;

  void gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data) override;
  void shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data) override;
  void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data) override;
};
}  // namespace rm_referee
