//
// Created by chen on 24-11-23.
//
#include "rm_vt/video_tran.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_vt");
  ros::NodeHandle nh("~");
  rm_vt::VideoTran video_tran(nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    video_tran.read();
    loop_rate.sleep();
  }
}
