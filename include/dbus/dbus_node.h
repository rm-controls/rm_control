//
// Created by qiayuan on 2019/10/30.
//

#ifndef SRC_RM_BRIDGE_INCLUDE_DBUS_NODE_H_
#define SRC_RM_BRIDGE_INCLUDE_DBUS_NODE_H_
#include "dbus/dbus.h"
#include <ros/ros.h>
#include <rm_msgs/DbusData.h>
class DBusNode {
 private:
  ros::NodeHandle nh_;
  ros::Publisher dbus_pub_;
  std::string serial_port_;
  rm_msgs::DbusData dbus_cmd_;
  DBus dbus_{};
 public:
  DBusNode();
  ~DBusNode() = default;
  void run();
};
#endif //SRC_RM_BRIDGE_INCLUDE_DBUS_NODE_H_
