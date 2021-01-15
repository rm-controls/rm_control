//
// Created by qiayuan on 2019/10/30.
//

#include "dbus/dbus_node.h"

int main(int argc, char **argv) {
  struct sched_param params{};
  params.sched_priority = 95;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    ROS_ERROR("[dbus] Set scheduler failed, RUN THIS NODE AS SUPER USER.\n");
  }
  ros::init(argc, argv, "dbus_node");
  DBusNode dbus_node;
  ros::Rate loop_rate(60);
  while (ros::ok()) {
    dbus_node.run();
    loop_rate.sleep();
  }
  return 0;
}

DBusNode::DBusNode() {
  dbus_pub_ = nh_.advertise<rm_msgs::DbusData>("dbus_data", 1);
  nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  dbus_.init(serial_port_.data());
}

void DBusNode::run() {
  dbus_.read();
  dbus_.getData(&dbus_cmd_);
  dbus_pub_.publish(dbus_cmd_);
}