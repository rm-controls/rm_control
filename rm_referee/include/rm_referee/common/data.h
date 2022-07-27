// Created by ljq on 2021/12/3.
//

#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64.h>

#include "rm_referee/referee/data.h"

namespace rm_referee
{
class Data
{
public:
  explicit Data(ros::NodeHandle& nh, Base& base) : base_(base), tf_listener_(tf_buffer_)
  {
  }

  // sub data
  uint8_t radar_data_;
  sensor_msgs::JointState joint_state_;
  rm_msgs::ActuatorState actuator_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::ChassisCmd chassis_cmd_data_;
  geometry_msgs::Twist vel2d_cmd_data_;
  rm_msgs::ShootCmd shoot_cmd_data_;
  rm_msgs::GimbalCmd gimbal_cmd_data_;
  rm_msgs::StateCmd card_cmd_data_;
  rm_msgs::EngineerCmd engineer_cmd_data_;
  rm_msgs::ManualToReferee manual_to_referee_data_;

  Base& base_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rm_referee
