//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee.h"
#include "rm_referee/referee/hero_referee.h"
#include "rm_referee/referee/standard_referee.h"
#include "rm_referee/referee/engineer_referee.h"
#include "rm_referee/referee/radar_referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "rm_referee");  // rm_referee
  ros::NodeHandle nh("~");
  rm_referee::Referee referee;
  robot = getParam(nh, "robot_type", static_cast<std::string>("error"));
  if (robot == "hero")
    referee.referee_ui_ = new rm_referee::HeroReferee(nh, referee.data_translation_);
  else if (robot == "sentry")
    referee.referee_ui_ = new rm_referee::RefereeBase(nh, referee.data_translation_);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(80);
  while (ros::ok())
  {
    ros::spinOnce();
    referee.read();
    loop_rate.sleep();
  }

  return 0;
}
