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
#include <rm_msgs/StateCmd.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/StatusChangeRequest.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/MultiDofCmd.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <control_msgs/JointControllerState.h>

#include "rm_common/ros_utilities.h"
#include "rm_common/decision/heat_limit.h"
#include "rm_common/decision/power_limit.h"
#include "rm_common/linear_interpolation.h"
#include "rm_common/filters/filters.h"

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
  virtual void updateGameRobotStatus(const rm_msgs::GameRobotStatus data)
  {
  }
  virtual void updateGameStatus(const rm_msgs::GameStatus data)
  {
  }
  virtual void updateCapacityData(const rm_msgs::CapacityData data)
  {
  }
  virtual void updatePowerHeatData(const rm_msgs::PowerHeatData data)
  {
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
  explicit TimeStampCommandSenderBase(ros::NodeHandle& nh) : CommandSenderBase<MsgType>(nh)
  {
  }
  void sendCommand(const ros::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
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
    XmlRpc::XmlRpcValue xml_rpc_value;
    if (!nh.getParam("max_linear_x", xml_rpc_value))
      ROS_ERROR("Max X linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      max_linear_x_.init(xml_rpc_value);
    if (!nh.getParam("max_linear_y", xml_rpc_value))
      ROS_ERROR("Max Y linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      max_linear_y_.init(xml_rpc_value);
    if (!nh.getParam("max_angular_z", xml_rpc_value))
      ROS_ERROR("Max Z angular velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      max_angular_z_.init(xml_rpc_value);
    std::string topic;
    nh.getParam("power_limit_topic", topic);
    power_limit_subscriber_ =
        nh.subscribe<rm_msgs::ChassisCmd>(topic, 1, &Vel2DCommandSender::powerLimitCallback, this);
  }

  void setLinearXVel(double scale)
  {
    msg_.linear.x = scale * max_linear_x_.output(power_limit_);
  };
  void setLinearYVel(double scale)
  {
    msg_.linear.y = scale * max_linear_y_.output(power_limit_);
  };
  void setAngularZVel(double scale)
  {
    msg_.angular.z = scale * max_angular_z_.output(power_limit_);
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
  void powerLimitCallback(const rm_msgs::ChassisCmd::ConstPtr& msg)
  {
    power_limit_ = msg->power_limit;
  }
  LinearInterp max_linear_x_, max_linear_y_, max_angular_z_;
  double power_limit_ = 0;
  ros::Subscriber power_limit_subscriber_;
};

class ChassisCommandSender : public TimeStampCommandSenderBase<rm_msgs::ChassisCmd>
{
public:
  explicit ChassisCommandSender(ros::NodeHandle& nh) : TimeStampCommandSenderBase<rm_msgs::ChassisCmd>(nh)
  {
    XmlRpc::XmlRpcValue xml_rpc_value;
    power_limit_ = new PowerLimit(nh);
    if (!nh.getParam("accel_x", xml_rpc_value))
      ROS_ERROR("Accel X no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      accel_x_.init(xml_rpc_value);
    if (!nh.getParam("accel_y", xml_rpc_value))
      ROS_ERROR("Accel Y no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      accel_y_.init(xml_rpc_value);
    if (!nh.getParam("accel_z", xml_rpc_value))
      ROS_ERROR("Accel Z no defined (namespace: %s)", nh.getNamespace().c_str());
    else
      accel_z_.init(xml_rpc_value);
  }

  void updateGameStatus(const rm_msgs::GameStatus data) override
  {
    power_limit_->setGameProgress(data);
  }
  void updateGameRobotStatus(const rm_msgs::GameRobotStatus data) override
  {
    power_limit_->setGameRobotData(data);
  }
  void updatePowerHeatData(const rm_msgs::PowerHeatData data) override
  {
    power_limit_->setChassisPowerBuffer(data);
  }
  void updateCapacityData(const rm_msgs::CapacityData data) override
  {
    power_limit_->setCapacityData(data);
  }
  void updateRefereeStatus(bool status)
  {
    power_limit_->setRefereeStatus(status);
  }
  void sendChassisCommand(const ros::Time& time, bool is_gyro)
  {
    power_limit_->setLimitPower(msg_, is_gyro);
    msg_.accel.linear.x = accel_x_.output(msg_.power_limit);
    msg_.accel.linear.y = accel_y_.output(msg_.power_limit);
    msg_.accel.angular.z = accel_z_.output(msg_.power_limit);
    TimeStampCommandSenderBase<rm_msgs::ChassisCmd>::sendCommand(time);
  }
  void setZero() override{};
  PowerLimit* power_limit_;

private:
  LinearInterp accel_x_, accel_y_, accel_z_;
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm_msgs::GimbalCmd>
{
public:
  explicit GimbalCommandSender(ros::NodeHandle& nh) : TimeStampCommandSenderBase<rm_msgs::GimbalCmd>(nh)
  {
    if (!nh.getParam("max_yaw_vel", max_yaw_rate_))
      ROS_ERROR("Max yaw velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_pitch_vel", max_pitch_vel_))
      ROS_ERROR("Max pitch velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("track_timeout", track_timeout_))
      ROS_ERROR("Track timeout no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("eject_sensitivity", eject_sensitivity_))
      eject_sensitivity_ = 1.;
  }
  ~GimbalCommandSender() = default;
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
  void setEject(bool flag)
  {
    eject_flag_ = flag;
  }
  bool getEject() const
  {
    return eject_flag_;
  }
  void setPoint(geometry_msgs::PointStamped point)
  {
    msg_.target_pos = point;
  }

private:
  double max_yaw_rate_{}, max_pitch_vel_{}, track_timeout_{}, eject_sensitivity_ = 1.;
  bool eject_flag_{};
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd>
{
public:
  explicit ShooterCommandSender(ros::NodeHandle& nh) : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh)
  {
    ros::NodeHandle limit_nh(nh, "heat_limit");
    heat_limit_ = new HeatLimit(limit_nh);
    nh.param("speed_10m_per_speed", speed_10_, 10.);
    nh.param("speed_15m_per_speed", speed_15_, 15.);
    nh.param("speed_16m_per_speed", speed_16_, 16.);
    nh.param("speed_18m_per_speed", speed_18_, 18.);
    nh.param("speed_30m_per_speed", speed_30_, 30.);
    if (!nh.getParam("gimbal_error_tolerance", gimbal_error_tolerance_))
      ROS_ERROR("gimbal error tolerance no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("target_acceleration_tolerance", target_acceleration_tolerance_))
    {
      target_acceleration_tolerance_ = 0.;
      ROS_INFO("target_acceleration_tolerance no defined(namespace: %s), set to zero.", nh.getNamespace().c_str());
    }
  }
  ~ShooterCommandSender()
  {
    delete heat_limit_;
  }

  void updateGameRobotStatus(const rm_msgs::GameRobotStatus data) override
  {
    heat_limit_->setStatusOfShooter(data);
  }
  void updatePowerHeatData(const rm_msgs::PowerHeatData data) override
  {
    heat_limit_->setCoolingHeatOfShooter(data);
  }
  void updateRefereeStatus(bool status)
  {
    heat_limit_->setRefereeStatus(status);
  }
  void updateGimbalDesError(const rm_msgs::GimbalDesError& error)
  {
    gimbal_des_error_ = error;
  }
  void updateTrackData(const rm_msgs::TrackData& data)
  {
    track_data_ = data;
  }
  void updateSuggestFireData(const std_msgs::Bool& data)
  {
    suggest_fire_ = data;
  }
  void checkError(const ros::Time& time)
  {
    if (((gimbal_des_error_.error > gimbal_error_tolerance_ && time - gimbal_des_error_.stamp < ros::Duration(0.1)) ||
         (track_data_.accel > target_acceleration_tolerance_)) ||
        (!suggest_fire_.data && armor_type_ == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE))
      if (msg_.mode == rm_msgs::ShootCmd::PUSH)
        setMode(rm_msgs::ShootCmd::READY);
  }
  void sendCommand(const ros::Time& time) override
  {
    msg_.speed = heat_limit_->getSpeedLimit();
    msg_.hz = heat_limit_->getShootFrequency();
    TimeStampCommandSenderBase<rm_msgs::ShootCmd>::sendCommand(time);
  }
  double getSpeed()
  {
    switch (msg_.speed)
    {
      case rm_msgs::ShootCmd::SPEED_10M_PER_SECOND:
        return speed_10_;
      case rm_msgs::ShootCmd::SPEED_15M_PER_SECOND:
        return speed_15_;
      case rm_msgs::ShootCmd::SPEED_16M_PER_SECOND:
        return speed_16_;
      case rm_msgs::ShootCmd::SPEED_18M_PER_SECOND:
        return speed_18_;
      case rm_msgs::ShootCmd::SPEED_30M_PER_SECOND:
        return speed_30_;
    }
    return 0.;
  }
  void setArmorType(uint8_t armor_type)
  {
    armor_type_ = armor_type;
  }
  void setShootFrequency(uint8_t mode)
  {
    heat_limit_->setShootFrequency(mode);
  }
  uint8_t getShootFrequency()
  {
    return heat_limit_->getShootFrequencyMode();
  }
  void setZero() override{};
  HeatLimit* heat_limit_{};

private:
  double speed_10_, speed_15_, speed_16_, speed_18_, speed_30_;
  double gimbal_error_tolerance_{};
  double target_acceleration_tolerance_{};
  rm_msgs::TrackData track_data_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  std_msgs::Bool suggest_fire_;
  uint8_t armor_type_{};
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

class CardCommandSender : public CommandSenderBase<std_msgs::Float64>
{
public:
  explicit CardCommandSender(ros::NodeHandle& nh) : CommandSenderBase<std_msgs::Float64>(nh)
  {
    ROS_ASSERT(nh.getParam("long_pos", long_pos_) && nh.getParam("short_pos", short_pos_) &&
               nh.getParam("off_pos", off_pos_));
  }
  void long_on()
  {
    msg_.data = long_pos_;
    state = true;
  }
  void short_on()
  {
    msg_.data = short_pos_;
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
  double long_pos_{}, short_pos_{}, off_pos_{};
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

class JointPointCommandSender : public CommandSenderBase<std_msgs::Float64>
{
public:
  explicit JointPointCommandSender(ros::NodeHandle& nh, const sensor_msgs::JointState& joint_state)
    : CommandSenderBase<std_msgs::Float64>(nh), joint_state_(joint_state)
  {
    ROS_ASSERT(nh.getParam("joint", joint_));
  }
  void setPoint(double point)
  {
    msg_.data = point;
  }
  int getIndex()
  {
    auto i = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_);
    if (i != joint_state_.name.end())
    {
      index_ = std::distance(joint_state_.name.begin(), i);
      return index_;
    }
    else
    {
      ROS_ERROR("Can not find joint %s", joint_.c_str());
      return -1;
    }
  }
  void setZero() override{};

private:
  std::string joint_{};
  int index_{};
  const sensor_msgs::JointState& joint_state_;
};

class CameraSwitchCommandSender : public CommandSenderBase<std_msgs::String>
{
public:
  explicit CameraSwitchCommandSender(ros::NodeHandle& nh) : CommandSenderBase<std_msgs::String>(nh)
  {
    ROS_ASSERT(nh.getParam("camera1_name", camera1_name_) && nh.getParam("camera2_name", camera2_name_));
    msg_.data = camera1_name_;
  }
  void switchCamera()
  {
    msg_.data = msg_.data == camera1_name_ ? camera2_name_ : camera1_name_;
  }
  void sendCommand(const ros::Time& time) override
  {
    CommandSenderBase<std_msgs::String>::sendCommand(time);
  }
  void setZero() override{};

private:
  std::string camera1_name_{}, camera2_name_{};
};

class MultiDofCommandSender : public TimeStampCommandSenderBase<rm_msgs::MultiDofCmd>
{
public:
  explicit MultiDofCommandSender(ros::NodeHandle& nh) : TimeStampCommandSenderBase<rm_msgs::MultiDofCmd>(nh)
  {
  }
  ~MultiDofCommandSender() = default;
  void setMode(int mode)
  {
    msg_.mode = mode;
  }
  int getMode()
  {
    return msg_.mode;
  }
  void setGroupValue(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y,
                     double angular_z)
  {
    msg_.linear.x = linear_x;
    msg_.linear.y = linear_y;
    msg_.linear.z = linear_z;
    msg_.angular.x = angular_x;
    msg_.angular.y = angular_y;
    msg_.angular.z = angular_z;
  }
  void setZero() override
  {
    msg_.linear.x = 0;
    msg_.linear.y = 0;
    msg_.linear.z = 0;
    msg_.angular.x = 0;
    msg_.angular.y = 0;
    msg_.angular.z = 0;
  }

private:
  ros::Time time_;
};

class DoubleBarrelCommandSender
{
public:
  DoubleBarrelCommandSender(ros::NodeHandle& nh)
  {
    ros::NodeHandle shooter_ID1_nh(nh, "shooter_ID1");
    shooter_ID1_cmd_sender_ = new ShooterCommandSender(shooter_ID1_nh);
    ros::NodeHandle shooter_ID2_nh(nh, "shooter_ID2");
    shooter_ID2_cmd_sender_ = new ShooterCommandSender(shooter_ID2_nh);
    ros::NodeHandle barrel_nh(nh, "barrel");
    barrel_command_sender_ = new rm_common::JointPointCommandSender(barrel_nh, joint_state_);

    barrel_nh.getParam("is_double_barrel", is_double_barrel_);
    barrel_nh.getParam("id1_point", id1_point_);
    barrel_nh.getParam("id2_point", id2_point_);
    barrel_nh.getParam("restart_push_threshold", restart_push_threshold_);
    barrel_nh.getParam("cooling_threshold", cooling_threshold_);

    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10,
                                                             &DoubleBarrelCommandSender::jointStateCallback, this);
    trigger_state_sub_ = nh.subscribe<control_msgs::JointControllerState>(
        "/controllers/shooter_controller/trigger/state", 10, &DoubleBarrelCommandSender::triggerStateCallback, this);
  }

  void updateGameRobotStatus(const rm_msgs::GameRobotStatus data)
  {
    shooter_ID1_cmd_sender_->updateGameRobotStatus(data);
    shooter_ID2_cmd_sender_->updateGameRobotStatus(data);
  }
  void updatePowerHeatData(const rm_msgs::PowerHeatData data)
  {
    shooter_ID1_cmd_sender_->heat_limit_->setCoolingHeatOfShooter(data);
    shooter_ID2_cmd_sender_->heat_limit_->setCoolingHeatOfShooter(data);
  }
  void updateRefereeStatus(bool status)
  {
    shooter_ID1_cmd_sender_->updateRefereeStatus(status);
    shooter_ID2_cmd_sender_->updateRefereeStatus(status);
  }
  void updateGimbalDesError(const rm_msgs::GimbalDesError& error)
  {
    shooter_ID1_cmd_sender_->updateGimbalDesError(error);
    shooter_ID2_cmd_sender_->updateGimbalDesError(error);
  }
  void updateTrackData(const rm_msgs::TrackData& data)
  {
    shooter_ID1_cmd_sender_->updateTrackData(data);
    shooter_ID2_cmd_sender_->updateTrackData(data);
  }
  void updateSuggestFireData(const std_msgs::Bool& data)
  {
    shooter_ID1_cmd_sender_->updateSuggestFireData(data);
    shooter_ID2_cmd_sender_->updateSuggestFireData(data);
  }

  void setMode(int mode)
  {
    shooter_ID1_cmd_sender_->setMode(mode);
    shooter_ID2_cmd_sender_->setMode(mode);
  }
  void setZero()
  {
    shooter_ID1_cmd_sender_->setZero();
    shooter_ID2_cmd_sender_->setZero();
  }
  void checkError(const ros::Time& time)
  {
    shooter_ID1_cmd_sender_->checkError(time);
    shooter_ID2_cmd_sender_->checkError(time);
  }
  void sendCommand(const ros::Time& time)
  {
    if (doSwitch())
    {
      switch_barrel_ = true;
      switch_done_ = false;
    }
    if ((shooter_ID1_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::PUSH ||
         shooter_ID2_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::PUSH) &&
        switch_barrel_)
    {
      setMode(rm_msgs::ShootCmd::READY);
      if (!switch_done_)
        switchBarrel();
      else
        doRestartPush();
    }
    shooter_ID1_cmd_sender_->sendCommand(time);
    shooter_ID2_cmd_sender_->sendCommand(time);
  }
  int initBarrelId()
  {
    ros::Time time = ros::Time::now();
    barrel_id_ = 0;
    barrel_command_sender_->setPoint(id1_point_);
    barrel_command_sender_->sendCommand(time);
    return barrel_id_;
  }
  void setArmorType(uint8_t armor_type)
  {
    shooter_ID1_cmd_sender_->setArmorType(armor_type);
    shooter_ID2_cmd_sender_->setArmorType(armor_type);
  }
  void setShootFrequency(uint8_t mode)
  {
    shooter_ID1_cmd_sender_->setShootFrequency(mode);
    shooter_ID2_cmd_sender_->setShootFrequency(mode);
  }
  uint8_t getShootFrequency()
  {
    return shooter_ID1_cmd_sender_->getShootFrequency();
  }
  double getSpeed()
  {
    return shooter_ID1_cmd_sender_->getSpeed();
  }

private:
  int getBarrelId()
  {
    if (barrel_command_sender_->getMsg()->data == id1_point_)
      barrel_id_ = 0;
    else
      barrel_id_ = 1;
    return barrel_id_;
  }
  void switchBarrel()
  {
    ros::Time time = ros::Time::now();
    if (std::fmod(std::abs(trigger_error_), 2. * M_PI) < 0.01)
      switch_done_ = true;
    if (switch_done_)
    {
      barrel_command_sender_->getMsg()->data == id2_point_ ? barrel_command_sender_->setPoint(id1_point_) :
                                                             barrel_command_sender_->setPoint(id2_point_);
      barrel_command_sender_->sendCommand(time);
    }
  }
  void doRestartPush()
  {
    if (std::abs(joint_state_.position[barrel_command_sender_->getIndex()] - barrel_command_sender_->getMsg()->data) <
        restart_push_threshold_)
    {
      switch_barrel_ = false;
      setMode(rm_msgs::ShootCmd::PUSH);
    }
  }

  bool doSwitch()
  {
    if (!is_double_barrel_)
      return false;
    int shooter_ID1_cooling_limit = shooter_ID1_cmd_sender_->heat_limit_->getCoolingLimit();
    int shooter_ID2_cooling_limit = shooter_ID2_cmd_sender_->heat_limit_->getCoolingLimit();
    int shooter_ID1_cooling_heat = shooter_ID1_cmd_sender_->heat_limit_->getCoolingHeat();
    int shooter_ID2_cooling_heat = shooter_ID2_cmd_sender_->heat_limit_->getCoolingHeat();

    if (shooter_ID1_cooling_limit == 0 || shooter_ID2_cooling_limit == 0)
    {
      ROS_WARN("Can not get cooling limit");
      return false;
    }

    if (shooter_ID1_cooling_limit - shooter_ID1_cooling_heat < cooling_threshold_[0] ||
        shooter_ID2_cooling_limit - shooter_ID2_cooling_heat < cooling_threshold_[0])
    {
      if (getBarrelId())
        return shooter_ID2_cooling_limit - shooter_ID2_cooling_heat < cooling_threshold_[0] &&
               shooter_ID1_cooling_limit - shooter_ID1_cooling_heat > cooling_threshold_[1];
      else
        return shooter_ID1_cooling_limit - shooter_ID1_cooling_heat < cooling_threshold_[0] &&
               shooter_ID2_cooling_limit - shooter_ID2_cooling_heat > cooling_threshold_[1];
    }
    else
      return false;
  }
  void triggerStateCallback(const control_msgs::JointControllerState::ConstPtr& data)
  {
    trigger_error_ = data->error;
  }
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
  {
    joint_state_ = *data;
  }
  ShooterCommandSender* shooter_ID1_cmd_sender_;
  ShooterCommandSender* shooter_ID2_cmd_sender_;
  JointPointCommandSender* barrel_command_sender_{};
  ros::Subscriber trigger_state_sub_;
  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState joint_state_;
  bool is_double_barrel_{ false }, switch_barrel_{ false }, switch_done_{ true };
  double trigger_error_;
  int barrel_id_;
  double id1_point_, id2_point_;
  double restart_push_threshold_;
  std::vector<int> cooling_threshold_;
};

}  // namespace rm_common
