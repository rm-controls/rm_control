//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/common/referee_base.h"
#include "rm_referee/hero_referee.h"
#include "rm_referee/standard_referee.h"
#include "rm_referee/engineer_referee.h"
#include "rm_referee/radar_referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  rm_referee::RefereeBase* referee;
  ros::init(argc, argv, "rm_referee");  // rm_referee
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard3")
    referee = new rm_referee::StandardReferee(nh);
  else if (robot == "hero")
    referee = new rm_referee::HeroReferee(nh);
  else if (robot == "engineer")
    referee = new rm_referee::EngineerReferee(nh);
  else if (robot == "radar")
    referee = new rm_referee::RadarReferee(nh);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    referee->run();
    loop_rate.sleep();
  }

  return 0;
}
