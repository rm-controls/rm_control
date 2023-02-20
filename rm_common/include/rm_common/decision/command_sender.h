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
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>

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
class MultiDofCommandSender
{
public:
    explicit MultiDofCommandSender(ros::NodeHandle& nh)
    {
        uint32_t queue_size = getParam(nh, "queue_size", 1);
        ros::NodeHandle nh_pid_zero = ros::NodeHandle(nh, "pid_zero");
        std::string topic_joint1, topic_joint2, topic_joint3, topic_joint4;
        XmlRpc::XmlRpcValue roll_config{}, pitch_config{}, yaw_config{}, x_config{}, y_config{}, z_config{};
        ROS_ASSERT(nh.getParam("translate_max_speed", translate_max_speed_) &&
                   nh.getParam("reversal_max_speed", reversal_max_speed_));
        ROS_ASSERT(nh.getParam("topic_joint1", topic_joint1) && nh.getParam("topic_joint2", topic_joint2) &&
                   nh.getParam("topic_joint3", topic_joint3) && nh.getParam("topic_joint4", topic_joint4));
        if (nh.getParam("roll", roll_config))
        {
            for (int i = 0; i < roll_config.size(); ++i)
                roll_config_.push_back(xmlRpcGetDouble(roll_config[i]));
            ros::NodeHandle nh_pid_roll = ros::NodeHandle(nh, "pid_roll");
            pid_roll_.init(ros::NodeHandle(nh_pid_roll, "pid"));
        }
        else
        {
            roll_config_ = { 0., 0., 0., 0. };
            pid_roll_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        if (nh.getParam("pitch", pitch_config))
        {
            for (int i = 0; i < pitch_config.size(); ++i)
                pitch_config_.push_back(xmlRpcGetDouble(pitch_config[i]));
            ros::NodeHandle nh_pid_pitch = ros::NodeHandle(nh, "pid_pitch");
            pid_pitch_.init(ros::NodeHandle(nh_pid_pitch, "pid"));
        }
        else
        {
            pitch_config_ = { 0., 0., 0., 0. };
            pid_pitch_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        if (nh.getParam("yaw", yaw_config))
        {
            for (int i = 0; i < yaw_config.size(); ++i)
                yaw_config_.push_back(xmlRpcGetDouble(yaw_config[i]));
            ros::NodeHandle nh_pid_yaw = ros::NodeHandle(nh, "pid_yaw");
            pid_yaw_.init(ros::NodeHandle(nh_pid_yaw, "pid"));
        }
        else
        {
            yaw_config_ = { 0., 0., 0., 0. };
            pid_yaw_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        if (nh.getParam("x", x_config))
        {
            for (int i = 0; i < x_config.size(); ++i)
                x_config_.push_back(xmlRpcGetDouble(x_config[i]));
            ros::NodeHandle nh_pid_x = ros::NodeHandle(nh, "pid_x");
            pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
        }
        else
        {
            x_config_ = { 0., 0., 0., 0. };
            pid_x_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        if (nh.getParam("y", y_config))
        {
            for (int i = 0; i < y_config.size(); ++i)
                y_config_.push_back(xmlRpcGetDouble(y_config[i]));
            ros::NodeHandle nh_pid_y = ros::NodeHandle(nh, "pid_y");
            pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
        }
        else
        {
            y_config_ = { 0., 0., 0., 0. };
            pid_y_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        if (nh.getParam("z", z_config))
        {
            for (int i = 0; i < z_config.size(); ++i)
                z_config_.push_back(xmlRpcGetDouble(z_config[i]));
            ros::NodeHandle nh_pid_z = ros::NodeHandle(nh, "pid_z");
            pid_z_.init(ros::NodeHandle(nh_pid_z, "pid"));
        }
        else
        {
            z_config_ = { 0., 0., 0., 0. };
            pid_z_.init(ros::NodeHandle(nh_pid_zero, "pid"));
        }
        pub_joint1_ = nh.advertise<std_msgs::Float64>(topic_joint1, queue_size);
        pub_joint2_ = nh.advertise<std_msgs::Float64>(topic_joint2, queue_size);
        pub_joint3_ = nh.advertise<std_msgs::Float64>(topic_joint3, queue_size);
        pub_joint4_ = nh.advertise<std_msgs::Float64>(topic_joint4, queue_size);
    };
    void visionReversal(double error_roll, double error_pitch, double error_yaw, double error_x, double error_y,
                        double error_z, ros::Duration period)
    {
        double roll_scale = pid_roll_.computeCommand(error_roll, period);
        double pitch_scale = pid_pitch_.computeCommand(error_pitch, period);
        double yaw_scale = pid_yaw_.computeCommand(error_yaw, period);
        double x_scale = pid_x_.computeCommand(error_x, period);
        double y_scale = pid_y_.computeCommand(error_y, period);
        double z_scale = pid_z_.computeCommand(error_z, period);
        setGroupVel(roll_scale, pitch_scale, yaw_scale, x_scale, y_scale, z_scale);
    }
    void setGroupVel(double roll_scale, double pitch_scale, double yaw_scale, double x_scale, double y_scale,
                     double z_scale)
    {
        msg_joint1_.data =
                reversal_max_speed_ *
                ((roll_config_[0] * roll_scale) + (pitch_config_[0] * pitch_scale) + (yaw_config_[0] * yaw_scale)) +
                translate_max_speed_ * ((x_config_[0] * x_scale) + (y_config_[0] * y_scale) + (z_config_[0] * z_scale));
        msg_joint2_.data =
                reversal_max_speed_ *
                ((roll_config_[1] * roll_scale) + (pitch_config_[1] * pitch_scale) + (yaw_config_[1] * yaw_scale)) +
                translate_max_speed_ * ((x_config_[1] * x_scale) + (y_config_[1] * y_scale) + (z_config_[1] * z_scale));
        msg_joint3_.data =
                reversal_max_speed_ *
                ((roll_config_[2] * roll_scale) + (pitch_config_[2] * pitch_scale) + (yaw_config_[2] * yaw_scale)) +
                translate_max_speed_ * ((x_config_[2] * x_scale) + (y_config_[2] * y_scale) + (z_config_[2] * z_scale));
        msg_joint4_.data =
                reversal_max_speed_ *
                ((roll_config_[3] * roll_scale) + (pitch_config_[3] * pitch_scale) + (yaw_config_[3] * yaw_scale)) +
                translate_max_speed_ * ((x_config_[3] * x_scale) + (y_config_[3] * y_scale) + (z_config_[3] * z_scale));
    }
    void setZero()
    {
        msg_joint1_.data = 0;
        msg_joint2_.data = 0;
        msg_joint3_.data = 0;
        msg_joint4_.data = 0;
    }
    void sendCommand()
    {
        pub_joint1_.publish(msg_joint1_);
        pub_joint2_.publish(msg_joint2_);
        pub_joint3_.publish(msg_joint3_);
        pub_joint4_.publish(msg_joint4_);
    }

protected:
    double reversal_max_speed_, translate_max_speed_;
    ros::Publisher pub_joint1_, pub_joint2_, pub_joint3_, pub_joint4_;
    std::vector<double> roll_config_, pitch_config_, yaw_config_, x_config_, y_config_, z_config_;
    std_msgs::Float64 msg_joint1_{}, msg_joint2_{}, msg_joint3_{}, msg_joint4_{};
    control_toolbox::Pid pid_roll_, pid_pitch_, pid_yaw_, pid_x_, pid_y_, pid_z_;
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

  void sendCommand(const ros::Time& time) override
  {
    power_limit_->setLimitPower(msg_);
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

private:
  ros::Time last_track_;
  double max_yaw_rate_{}, max_pitch_vel_{}, track_timeout_{}, eject_sensitivity_ = 1.;
  bool eject_flag_{};
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd>
{
public:
  explicit ShooterCommandSender(ros::NodeHandle& nh, const rm_msgs::TrackData& track_data)
    : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh), track_data_(track_data)
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
    double moving_average_num;
    nh.param("accleration_moving_average_num", moving_average_num, 1.);
    acceleration_filter_ = new MovingAverageFilter<double>(moving_average_num);
    track_target_acceleration_ = 0.;
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

  void computeTargetAcceleration()
  {
    auto target_vel = track_data_.target_vel;
    double current_target_vel = sqrt(pow(target_vel.x, 2) + pow(target_vel.y, 2) + pow(target_vel.z, 2));
    double current_time = track_data_.header.stamp.toSec();
    if (current_time == last_target_time_)
      return;
    track_target_acceleration_ = (current_target_vel - last_target_vel_) / (current_time - last_target_time_);
    last_target_vel_ = current_target_vel;
    last_target_time_ = current_time;
    acceleration_filter_->input(track_target_acceleration_);
    track_target_acceleration_ = acceleration_filter_->output();
  }
  void checkError(const rm_msgs::GimbalDesError& gimbal_des_error, const ros::Time& time)
  {
    if ((gimbal_des_error.error > gimbal_error_tolerance_ && time - gimbal_des_error.stamp < ros::Duration(0.1)) ||
        (track_target_acceleration_ > target_acceleration_tolerance_))
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
  double track_target_acceleration_;
  double last_target_vel_ = 0.;
  double last_target_time_ = 0.;
  const rm_msgs::TrackData& track_data_;
  MovingAverageFilter<double>* acceleration_filter_;
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
}  // namespace rm_common
