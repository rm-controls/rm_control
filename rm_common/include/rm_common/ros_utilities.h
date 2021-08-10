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

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue &value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    const int tmp = value;
    return (double) tmp;
  } else
    return value;
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue &value, int field) {
  ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
      (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
  XmlRpc::XmlRpcValue value_xml = value[field];
  return xmlRpcGetDouble(value[field]);
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue &value, const std::string &field, double default_value) {
  if (value.hasMember(field)) {
    ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
        (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    return xmlRpcGetDouble(value[field]);
  } else
    return default_value;
}

#endif // RM_COMMON_ROS_UTILITIES_H
