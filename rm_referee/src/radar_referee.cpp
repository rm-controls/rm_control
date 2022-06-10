//
// Created by yuchen on 2022/6/10.
//

#include "rm_referee/radar_referee.h"

namespace rm_referee
{
RadarReferee::RadarReferee(ros::NodeHandle& nh) : RefereeBase(nh)
{
  ros::NodeHandle ui_nh(nh, "radar");
  radar_date_sub_ = nh.subscribe<std_msgs::Int8MultiArray>("/data", 10, &RadarReferee::radarDataCallBack, this);
}
void RadarReferee::run()
{
  RefereeBase::run();
}
}  // namespace rm_referee