//
// Created by yuchen on 2022/6/10.
//

#include "rm_referee/radar_referee.h"

namespace rm_referee
{
RadarReferee::RadarReferee(ros::NodeHandle& nh) : RefereeBase(nh)
{
  ros::NodeHandle ui_nh(nh, "radar");
}
void RadarReferee::run()
{
  RefereeBase::run();
  for (int target :
       data_.referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_RADAR ? red_receiver : blue_receiver)
  {
    data_.referee_.sendInteractiveData(0x0202, target, data_.radar_data_);
  }
}
}  // namespace rm_referee