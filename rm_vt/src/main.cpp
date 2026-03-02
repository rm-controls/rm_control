//
// Created by ch on 24-11-23.
//
#include "rm_vt/video_transmission.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_vt");
  ros::NodeHandle nh("~");
  rm_vt::VideoTransmission video_transmission(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    video_transmission.read();
    loop_rate.sleep();
  }
}
