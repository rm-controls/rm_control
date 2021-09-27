/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 5/18/21.
//

#pragma once

#include <type_traits>
#include <utility>

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/GimbalDesError.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "rm_common/ros_utilities.h"
#include "rm_common/decision/heat_limit.h"
#include "rm_common/decision/power_limit.h"
#include "rm_common/decision/target_cost_function.h"

namespace rm_common
{
template <class MsgType>
class CommandSenderBase
{
public:
  explicit CommandSenderBase(ros::NodeHandle& nh)
  {
    if (!nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined (namespace: %s)", nh.getNamespace().c_str());
    queue_size_ = getParam(nh, "queue_size", 1);
    pub_ = nh.advertise<MsgType>(topic_, queue_size_);
  }
  void setMode(int mode)
  {
    if (!std::is_same<MsgType, geometry_msgs::Twist>::value && !std::is_same<MsgType, std_msgs::Float64>::value)
      msg_.mode = mode;
  }
  virtual void sendCommand(const ros::Time& time)
  {
    pub_.publish(msg_);
  }
  virtual void setZero() = 0;
  MsgType* getMsg()
  {
    return &msg_;
  }

protected:
  std::string topic_;
  uint32_t queue_size_;
  ros::Publisher pub_;
  MsgType msg_;
};

template <class MsgType>
class TimeStampCommandSenderBase : public CommandSenderBase<MsgType>
{
public:
  explicit TimeStampCommandSenderBase(ros::NodeHandle& nh, const RefereeData& referee_data)
    : CommandSenderBase<MsgType>(nh), referee_data_(referee_data)
  {
  }
  void sendCommand(const ros::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }

protected:
  const RefereeData& referee_data_;
};

template <class MsgType>
class HeaderStampCommandSenderBase : public CommandSenderBase<MsgType>
{
public:
  explicit HeaderStampCommandSenderBase(ros::NodeHandle& nh) : CommandSenderBase<MsgType>(nh)
  {
  }
  void sendCommand(const ros::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.header.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

class Vel2DCommandSender : public CommandSenderBase<geometry_msgs::Twist>
{
public:
  explicit Vel2DCommandSender(ros::NodeHandle& nh) : CommandSenderBase<geometry_msgs::Twist>(nh)
  {
    if (!nh.getParam("max_linear_x", max_linear_x_))
      ROS_ERROR("Max X linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_linear_y", max_linear_y_))
      ROS_ERROR("Max Y linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_angular_z", max_angular_z_))
      ROS_ERROR("Max Z angular velocity no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  void setLinearXVel(double scale)
  {
    msg_.linear.x = scale * max_linear_x_;
  };
  void setLinearYVel(double scale)
  {
    msg_.linear.y = scale * max_linear_y_;
  };
  void setAngularZVel(double scale)
  {
    msg_.angular.z = scale * max_angular_z_;
  };
  void set2DVel(double scale_x, double scale_y, double scale_z)
  {
    setLinearXVel(scale_x);
    setLinearYVel(scale_y);
    setAngularZVel(scale_z);
  }
  void setZero() override
  {
    msg_.linear.x = 0.;
    msg_.linear.y = 0.;
    msg_.angular.z = 0.;
  }

protected:
  double max_linear_x_{}, max_linear_y_{}, max_angular_z_{};
};

class ChassisCommandSender : public TimeStampCommandSenderBase<rm_msgs::ChassisCmd>
{
public:
  explicit ChassisCommandSender(ros::NodeHandle& nh, const RefereeData& referee_data)
    : TimeStampCommandSenderBase<rm_msgs::ChassisCmd>(nh, referee_data)
  {
    double accel_x, accel_y, accel_z;
    power_limit_ = new PowerLimit(nh, referee_data, msg_);
    if (!nh.getParam("accel_x", accel_x))
      ROS_ERROR("Accel X no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_y", accel_y))
      ROS_ERROR("Accel Y no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_z", accel_z))
      ROS_ERROR("Accel Z no defined (namespace: %s)", nh.getNamespace().c_str());
    msg_.accel.linear.x = accel_x;
    msg_.accel.linear.y = accel_y;
    msg_.accel.angular.z = accel_z;
  }
  void sendCommand(const ros::Time& time) override
  {
    msg_.power_limit = power_limit_->getLimitPower();
    TimeStampCommandSenderBase<rm_msgs::ChassisCmd>::sendCommand(time);
  }
  void setZero() override{};
  PowerLimit* power_limit_;
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm_msgs::GimbalCmd>
{
public:
  explicit GimbalCommandSender(ros::NodeHandle& nh, const RefereeData& referee_data)
    : TimeStampCommandSenderBase<rm_msgs::GimbalCmd>(nh, referee_data)
  {
    if (!nh.getParam("max_yaw_vel", max_yaw_rate_))
      ROS_ERROR("Max yaw velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_pitch_vel", max_pitch_vel_))
      ROS_ERROR("Max pitch velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("track_timeout", track_timeout_))
      ROS_ERROR("Track timeout no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("eject_sensitivity", eject_sensitivity_))
      eject_sensitivity_ = 1.;
    cost_function_ = new TargetCostFunction(nh, referee_data);
  }
  ~GimbalCommandSender()
  {
    delete cost_function_;
  };
  void setRate(double scale_yaw, double scale_pitch)
  {
    msg_.rate_yaw = scale_yaw * max_yaw_rate_;
    msg_.rate_pitch = scale_pitch * max_pitch_vel_;
    if (eject_flag_)
    {
      msg_.rate_yaw *= eject_sensitivity_;
      msg_.rate_pitch *= eject_sensitivity_;
    }
  }
  void setZero() override
  {
    msg_.rate_yaw = 0.;
    msg_.rate_pitch = 0.;
  }
  void setBulletSpeed(double bullet_speed)
  {
    msg_.bullet_speed = bullet_speed;
  }
  void updateCost(const rm_msgs::TrackDataArray& track_data_array)
  {
    double cost = cost_function_->costFunction(track_data_array);
    if (!track_data_array.tracks.empty())
      last_track_ = ros::Time::now();
    if (cost == 1e9 && (ros::Time::now() - last_track_).toSec() > track_timeout_)
      setMode(rm_msgs::GimbalCmd::RATE);
    else
      setMode(rm_msgs::GimbalCmd::TRACK);
  }
  void setEject(bool flag)
  {
    eject_flag_ = flag;
  }
  bool getEject() const
  {
    return eject_flag_;
  }

private:
  TargetCostFunction* cost_function_;
  ros::Time last_track_;
  double max_yaw_rate_{}, max_pitch_vel_{}, track_timeout_{}, eject_sensitivity_ = 1.;
  bool eject_flag_{};
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd>
{
public:
  explicit ShooterCommandSender(ros::NodeHandle& nh, const RefereeData& referee_data)
    : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh, referee_data)
  {
    ros::NodeHandle limit_nh(nh, "heat_limit");
    heat_limit_ = new HeatLimit(limit_nh, referee_data);
    if (!nh.getParam("gimbal_error_limit", gimbal_error_limit_))
      ROS_ERROR("gimbal error limit no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  ~ShooterCommandSender()
  {
    delete heat_limit_;
  }
  void checkError(const rm_msgs::GimbalDesError& gimbal_des_error, const ros::Time& time)
  {
    if (gimbal_des_error.error > gimbal_error_limit_ && time - gimbal_des_error.stamp < ros::Duration(0.1))
      if (msg_.mode == rm_msgs::ShootCmd::PUSH)
        setMode(rm_msgs::ShootCmd::READY);
  }
  void sendCommand(const ros::Time& time) override
  {
    msg_.speed = heat_limit_->getSpeedLimit();
    msg_.hz = heat_limit_->getHz();
    TimeStampCommandSenderBase<rm_msgs::ShootCmd>::sendCommand(time);
  }
  double getSpeed()
  {
    switch (msg_.speed)
    {
      case rm_msgs::ShootCmd::SPEED_10M_PER_SECOND:
        return 10.;
      case rm_msgs::ShootCmd::SPEED_15M_PER_SECOND:
        return 15.;
      case rm_msgs::ShootCmd::SPEED_16M_PER_SECOND:
        return 16.;
      case rm_msgs::ShootCmd::SPEED_18M_PER_SECOND:
        return 18.;
      case rm_msgs::ShootCmd::SPEED_30M_PER_SECOND:
        return 30.;
    }
    return 0.;
  }
  void setBurstMode(bool burst_flag)
  {
    heat_limit_->setMode(burst_flag);
  }
  bool getBurstMode()
  {
    return heat_limit_->getMode();
  }
  void setZero() override{};

private:
  double gimbal_error_limit_{};
  HeatLimit* heat_limit_{};
};

class Vel3DCommandSender : public HeaderStampCommandSenderBase<geometry_msgs::TwistStamped>
{
public:
  explicit Vel3DCommandSender(ros::NodeHandle& nh) : HeaderStampCommandSenderBase(nh)
  {
    if (!nh.getParam("max_linear_x", max_linear_x_))
      ROS_ERROR("Max X linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_linear_y", max_linear_y_))
      ROS_ERROR("Max Y linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_linear_z", max_linear_z_))
      ROS_ERROR("Max Z linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_angular_x", max_angular_x_))
      ROS_ERROR("Max X linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_angular_y", max_angular_y_))
      ROS_ERROR("Max Y angular velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_angular_z", max_angular_z_))
      ROS_ERROR("Max Z angular velocity no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  void setLinearVel(double scale_x, double scale_y, double scale_z)
  {
    msg_.twist.linear.x = max_linear_x_ * scale_x;
    msg_.twist.linear.y = max_linear_y_ * scale_y;
    msg_.twist.linear.z = max_linear_z_ * scale_z;
  }
  void setAngularVel(double scale_x, double scale_y, double scale_z)
  {
    msg_.twist.angular.x = max_angular_x_ * scale_x;
    msg_.twist.angular.y = max_angular_y_ * scale_y;
    msg_.twist.angular.z = max_angular_z_ * scale_z;
  }
  void setZero() override
  {
    msg_.twist.linear.x = 0.;
    msg_.twist.linear.y = 0.;
    msg_.twist.linear.z = 0.;
    msg_.twist.angular.x = 0.;
    msg_.twist.angular.y = 0.;
    msg_.twist.angular.z = 0.;
  }

private:
  double max_linear_x_{}, max_linear_y_{}, max_linear_z_{}, max_angular_x_{}, max_angular_y_{}, max_angular_z_{};
};

class JointPositionBinaryCommandSender : public CommandSenderBase<std_msgs::Float64>
{
public:
  explicit JointPositionBinaryCommandSender(ros::NodeHandle& nh) : CommandSenderBase<std_msgs::Float64>(nh)
  {
    ROS_ASSERT(nh.getParam("on_pos", on_pos_) && nh.getParam("off_pos", off_pos_));
  }
  void on()
  {
    msg_.data = on_pos_;
    state = true;
  }
  void off()
  {
    msg_.data = off_pos_;
    state = false;
  }
  bool getState() const
  {
    return state;
  }
  void sendCommand(const ros::Time& time) override
  {
    CommandSenderBase<std_msgs::Float64>::sendCommand(time);
  }
  void setZero() override{};

private:
  bool state{};
  double on_pos_{}, off_pos_{};
};

class JointJogCommandSender : public CommandSenderBase<std_msgs::Float64>
{
public:
  explicit JointJogCommandSender(ros::NodeHandle& nh, const sensor_msgs::JointState& joint_state)
    : CommandSenderBase<std_msgs::Float64>(nh), joint_state_(joint_state)
  {
    ROS_ASSERT(nh.getParam("joint", joint_));
    ROS_ASSERT(nh.getParam("step", step_));
  }
  void reset()
  {
    auto i = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_);
    if (i != joint_state_.name.end())
      msg_.data = joint_state_.position[std::distance(joint_state_.name.begin(), i)];
    else
      msg_.data = NAN;
  }
  void plus()
  {
    if (msg_.data != NAN)
    {
      msg_.data += step_;
      sendCommand(ros::Time());
    }
  }
  void minus()
  {
    if (msg_.data != NAN)
    {
      msg_.data -= step_;
      sendCommand(ros::Time());
    }
  }
  const std::string& getJoint()
  {
    return joint_;
  }

private:
  std::string joint_{};
  const sensor_msgs::JointState& joint_state_;
  double step_{};
};
}  // namespace rm_common
