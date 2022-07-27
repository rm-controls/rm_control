//
// Created by yuchen on 2022/6/10.
//

#include "rm_referee/radar_referee.h"

namespace rm_referee
{
RadarReferee::RadarReferee(ros::NodeHandle& nh, Data& data) : RefereeBase(nh, data)
{
  ros::NodeHandle ui_nh(nh, "radar");
  interactive_data_sender_ = new Graph(data.base_);
}
void RadarReferee::run()
{
  RefereeBase::run();
  for (int target : data_.base_.robot_id_ == rm_referee::RobotId::RED_RADAR ? red_receiver_ : blue_receiver_)
  {
    interactive_data_sender_->sendInteractiveData(0x0202, target, data_.radar_data_);
  }
}
}  // namespace rm_referee
