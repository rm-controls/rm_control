//
// Created by qiayuan on 7/7/20.
//

#ifndef RM_COMMON_ROS_UTILITIES_H
#define RM_COMMON_ROS_UTILITIES_H
#include <ros/ros.h>
#include <XmlRpcException.h>

template<typename T>
T getParam(ros::NodeHandle &pnh,
           const std::string &param_name, const T &default_val) {
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

inline double xmlRpcGetDouble(const XmlRpc::XmlRpcValue &value, const std::string &field, double default_value) {
  if (value.hasMember(field)) {
    ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
        (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    if (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      int tmp = value[field];
      return (double) tmp;
    } else {
      return value[field];
    }
  } else {
    return default_value;
  }
}
inline double xmlRpcGetDouble(const XmlRpc::XmlRpcValue &value, int field, double default_value) {
  ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
      (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
  XmlRpc::XmlRpcValue value_xml = value[field];
  if (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    const int tmp = value_xml;
    return (double) tmp;
  } else {
    return value_xml;
  }
}
#endif // RM_COMMON_ROS_UTILITIES_H
