//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "rm_referee");  // rm_referee
  ros::NodeHandle nh("~");
  rm_referee::Referee referee;
  robot = getParam(nh, "robot_type", static_cast<std::string>("error"));
  ROS_INFO("HI12");
  if (robot == "standard")
  {
    ROS_INFO("HI1342");
    referee.referee_ui_ = new rm_referee::RefereeBase(nh, referee.base_);
    ROS_INFO("HI");
  }
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(80);
  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("HI23");
    referee.read();
    ROS_INFO("HI2443");
    loop_rate.sleep();
  }

  return 0;
}
