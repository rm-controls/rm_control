//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee/referee.h"
#include "rm_referee/hero_referee.h"
#include "rm_referee/standard_referee.h"
#include "rm_referee/engineer_referee.h"
#include "rm_referee/radar_referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "rm_referee");  // rm_referee
  ros::NodeHandle nh("~");
  rm_referee::Referee referee;
  rm_referee::Data data(nh, referee.base_);
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard")
    referee.referee_ui_ = new rm_referee::StandardReferee(nh, data);
  else if (robot == "hero")
    referee.referee_ui_ = new rm_referee::HeroReferee(nh, data);
  else if (robot == "engineer")
    referee.referee_ui_ = new rm_referee::EngineerReferee(nh, data);
  else if (robot == "radar")
    referee.referee_ui_ = new rm_referee::RadarReferee(nh, data);
  else if (robot == "sentry")
    referee.referee_ui_ = new rm_referee::RefereeBase(nh, data);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    ros::spinOnce();
    referee.read();
    loop_rate.sleep();
  }

  return 0;
}
