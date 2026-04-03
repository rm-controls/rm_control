//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "rm_referee");  // rm_referee
  ros::NodeHandle nh("~");
  rm_referee::Referee referee(nh);
  ros::Rate loop_rate(80);
  try
  {
    while (ros::ok())
    {
      ros::spinOnce();
      referee.read();
      loop_rate.sleep();
    }
  }
  catch (const serial::SerialException& e)
  {
    ROS_ERROR_STREAM("Serial lost: " << e.what());
    referee.reconnect();
  }
  return 0;
}
