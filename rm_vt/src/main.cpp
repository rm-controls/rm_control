//
// Created by ch on 24-11-23.
//
#include <Eigen/Geometry>
#include "rm_vt/common/data.h"
#include "rm_vt/video_transmission.h"
#include "rm_vt/video_transmission_sender.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_vt");
  ros::NodeHandle nh("~");
  // rm_vt::VideoTransmission video_transmission(nh);
  rm_vt::Base base_;
  base_.initSerial();
  rm_vt::VideoTransmissionBase video_transmission_sender(nh, base_);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    // video_transmission.read();
    loop_rate.sleep();
  }
}
