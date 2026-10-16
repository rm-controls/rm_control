//
// Created by ch on 24-11-23.
//
#include <Eigen/Geometry>
#include "rm_vt/video_transmission.h"
#include "rm_vt/video_transmission_sender.h"
#include "rm_vt/common/data.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_vt");
  ros::NodeHandle nh("~");
  rm_vt::Base base;
  base.initSerial();
  rm_vt::VideoTransmission video_transmission(nh, base);
  rm_vt::VideoTransmissionSender video_transmission_sender(nh, base);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    video_transmission.read();
    loop_rate.sleep();
  }
}
