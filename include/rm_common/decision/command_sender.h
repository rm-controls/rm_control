//
// Created by qiayuan on 5/18/21.
//

#ifndef RM_COMMON_COMMAND_SENDER_H_
#define RM_COMMON_COMMAND_SENDER_H_
#include <type_traits>
#include <utility>

#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/GimbalDesError.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "rm_common/ros_utilities.h"
#include "rm_common/decision/heat_limit.h"
#include "rm_common/decision/target_cost_function.h"

namespace rm_common {

template<class MsgType>
class CommandSenderBase {
 public:
  explicit CommandSenderBase(ros::NodeHandle &nh) {
    if (!nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined (namespace: %s)", nh.getNamespace().c_str());
    queue_size_ = getParam(nh, "queue_size", 1);
    pub_ = nh.advertise<MsgType>(topic_, queue_size_);
  }
  void setMode(int mode) {
    if (!std::is_same<MsgType, geometry_msgs::Twist>::value &&
        !std::is_same<MsgType, std_msgs::Float64>::value)
      msg_.mode = mode;
  }
  virtual void sendCommand(const ros::Time &time) { pub_.publish(msg_); }
  virtual void setZero() = 0;
  MsgType *getMsg() { return &msg_; }
 protected:
  std::string topic_;
  uint32_t queue_size_;
  ros::Publisher pub_;
  MsgType msg_;
};

template<class MsgType>
class TimeStampCommandSenderBase : public CommandSenderBase<MsgType> {
 public:
  explicit TimeStampCommandSenderBase(ros::NodeHandle &nh, const RefereeData &referee_data) :
      CommandSenderBase<MsgType>(nh), referee_data_(referee_data) {}
  void sendCommand(const ros::Time &time) override {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
 protected:
  const RefereeData &referee_data_;
};

template<class MsgType>
class HeaderStampCommandSenderBase : public CommandSenderBase<MsgType> {
 public:
  explicit HeaderStampCommandSenderBase(ros::NodeHandle &nh) :
      CommandSenderBase<MsgType>(nh) {}
  void sendCommand(const ros::Time &time) override {
    CommandSenderBase<MsgType>::msg_.header.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

class Vel2DCommandSender : public CommandSenderBase<geometry_msgs::Twist> {
 public:
  explicit Vel2DCommandSender(ros::NodeHandle &nh) : CommandSenderBase<geometry_msgs::Twist>(nh) {
    if (!nh.getParam("max_linear_x", max_linear_x_))
      ROS_ERROR("Max X linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_linear_y", max_linear_y_))
      ROS_ERROR("Max Y linear velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_angular_z", max_angular_z_))
      ROS_ERROR("Max Z angular velocity no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  void setLinearXVel(double scale) { msg_.linear.x = scale * max_linear_x_; };
  void setLinearYVel(double scale) { msg_.linear.y = scale * max_linear_y_; };
  void setAngularZVel(double scale) { msg_.angular.z = scale * max_angular_z_; };
  void set2DVel(double scale_x, double scale_y, double scale_z) {
    setLinearXVel(scale_x);
    setLinearYVel(scale_y);
    setAngularZVel(scale_z);
  }
  void setZero() override {
    msg_.linear.x = 0.;
    msg_.linear.y = 0.;
    msg_.angular.z = 0.;
  }

 protected:
  double max_linear_x_{}, max_linear_y_{}, max_angular_z_{};
};

class ChassisCommandSender : public TimeStampCommandSenderBase<rm_msgs::ChassisCmd> {
 public:
  explicit ChassisCommandSender(ros::NodeHandle &nh, const RefereeData &referee_data)
      : TimeStampCommandSenderBase<rm_msgs::ChassisCmd>(nh, referee_data) {
    double accel_x, accel_y, accel_z;
    if (!nh.getParam("accel_x", accel_x))
      ROS_ERROR("Accel X no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_y", accel_y))
      ROS_ERROR("Accel Y no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_z", accel_z))
      ROS_ERROR("Accel Z no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("safety_power", safety_power_))
      ROS_ERROR("Safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("capacitor_threshold", capacitor_threshold_))
      ROS_ERROR("Capacitor threshold no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("charge_power", charge_power_))
      ROS_ERROR("Charge power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("extra_power", extra_power_))
      ROS_ERROR("Extra power no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_power", burst_power_))
      ROS_ERROR("Burst power no defined (namespace: %s)", nh.getNamespace().c_str());
    msg_.accel.linear.x = accel_x;
    msg_.accel.linear.y = accel_y;
    msg_.accel.angular.z = accel_z;
    burst_flag_ = false;
  }
  void sendCommand(const ros::Time &time) override {
    updateLimit();
    TimeStampCommandSenderBase<rm_msgs::ChassisCmd>::sendCommand(time);
  }
  void setZero() override {};
  void setBurstMode(bool burst_flag) { burst_flag_ = burst_flag; }
  bool getBurstMode() const { return burst_flag_; }
  void setChargeMode(bool charge_flag) { charge_flag_ = charge_flag; }
  bool getChargeMode() const { return charge_flag_; }

 private:
  void updateLimit() {
    if (referee_data_.robot_id_ == rm_common::RobotId::BLUE_SENTRY
        || referee_data_.robot_id_ == rm_common::RobotId::RED_SENTRY)
      msg_.power_limit = 30;
    else if (referee_data_.robot_id_ == rm_common::RobotId::RED_ENGINEER
        || referee_data_.robot_id_ == rm_common::RobotId::BLUE_ENGINEER)
      msg_.power_limit = 300;
    else {
      if (referee_data_.is_online_) {
        if (referee_data_.capacity_data.is_online_) {
          if (checkCalibra())
            return;
          if (referee_data_.game_robot_status_.chassis_power_limit_ > 120)
            msg_.power_limit = burst_power_;
          else {
            if (burst_flag_)
              burst();
            else if (charge_flag_)
              charge();
            else if (!burst_flag_ && (abs(
                referee_data_.capacity_data.limit_power_ - referee_data_.game_robot_status_.chassis_power_limit_)
                < 0.05))
              normal();
          }
        } else
          msg_.power_limit = referee_data_.game_robot_status_.chassis_power_limit_;
      } else
        msg_.power_limit = safety_power_;
    }
  }
  void charge() { msg_.power_limit = referee_data_.game_robot_status_.chassis_power_limit_ * 0.85; }
  void burst() {
    if (referee_data_.capacity_data.cap_power_ > capacitor_threshold_){
      if (msg_.mode == rm_msgs::ChassisCmd::GYRO)
        msg_.power_limit = referee_data_.game_robot_status_.chassis_power_limit_ + extra_power_;
      else
        msg_.power_limit = burst_power_;
    }else
      msg_.power_limit = referee_data_.game_robot_status_.chassis_power_limit_;
  }
  void normal() { msg_.power_limit = referee_data_.game_robot_status_.chassis_power_limit_; }
  bool checkCalibra() {
    if (referee_data_.capacity_data.limit_power_ == 0 && referee_data_.is_online_) {
      msg_.power_limit = 0;
      return true;
    } else
      return false;
  }

  double safety_power_{};
  double capacitor_threshold_{};
  double charge_power_{};
  double extra_power_{};
  double burst_power_{};
  bool burst_flag_ = false;
  bool charge_flag_ = false;
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm_msgs::GimbalCmd> {
 public:
  explicit GimbalCommandSender(ros::NodeHandle &nh, const RefereeData &referee_data) :
      TimeStampCommandSenderBase<rm_msgs::GimbalCmd>(nh, referee_data) {
    if (!nh.getParam("max_yaw_vel", max_yaw_rate_))
      ROS_ERROR("Max yaw velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_pitch_vel", max_pitch_vel_))
      ROS_ERROR("Max pitch velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("track_timeout", track_timeout_))
      ROS_ERROR("Track timeout no defined (namespace: %s)", nh.getNamespace().c_str());
    cost_function_ = new TargetCostFunction(nh, referee_data);
  }
  ~GimbalCommandSender() { delete cost_function_; };
  void setRate(double scale_yaw, double scale_pitch) {
    msg_.rate_yaw = scale_yaw * max_yaw_rate_;
    msg_.rate_pitch = scale_pitch * max_pitch_vel_;
  }
  void setBulletSpeed(double bullet_speed) {
    msg_.bullet_speed = bullet_speed;
  }
  void setAimPoint(geometry_msgs::PointStamped aim_point) {
    msg_.aim_point = std::move(aim_point);
  }
  void updateCost(const rm_msgs::TrackDataArray &track_data_array) {
    double cost = cost_function_->costFunction(track_data_array, base_only_);
    msg_.target_id = cost_function_->getId();
    if (!track_data_array.tracks.empty())
      last_track_ = ros::Time::now();
    if (cost == 1e9 && (ros::Time::now() - last_track_).toSec() > track_timeout_)
      setMode(rm_msgs::GimbalCmd::RATE);
    else
      setMode(rm_msgs::GimbalCmd::TRACK);
  }
  void setZero() override {
    msg_.rate_yaw = 0.;
    msg_.rate_pitch = 0.;
  }
  void setBaseOnly(bool base_only) { base_only_ = base_only; }
  bool getBaseOnly() const { return base_only_; }
 private:
  TargetCostFunction *cost_function_;
  double max_yaw_rate_{}, max_pitch_vel_{}, track_timeout_{};
  bool base_only_ = false;
  ros::Time last_track_;
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd> {
 public:
  explicit ShooterCommandSender(ros::NodeHandle &nh, const RefereeData &referee_data)
      : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh, referee_data) {
    ros::NodeHandle limit_nh(nh, "heat_limit");
    heat_limit_ = new HeatLimit(limit_nh, referee_data);
    if (!nh.getParam("gimbal_error_limit", gimbal_error_limit_))
      ROS_ERROR("gimbal error limit no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  ~ShooterCommandSender() { delete heat_limit_; }
  void setCover(bool is_open) { msg_.cover = is_open; }
  void checkError(const rm_msgs::GimbalDesError &gimbal_des_error, const ros::Time &time) {
    if (gimbal_des_error.error > gimbal_error_limit_ && time - gimbal_des_error.stamp < ros::Duration(0.1))
      if (msg_.mode == rm_msgs::ShootCmd::PUSH) setMode(rm_msgs::ShootCmd::READY);
  }
  void sendCommand(const ros::Time &time) override {
    msg_.speed = heat_limit_->getSpeedLimit();
    msg_.hz = heat_limit_->getHz();
    TimeStampCommandSenderBase<rm_msgs::ShootCmd>::sendCommand(time);
  }
  double getSpeed() {
    switch (msg_.speed) {
      case rm_msgs::ShootCmd::SPEED_10M_PER_SECOND: return 10.;
      case rm_msgs::ShootCmd::SPEED_15M_PER_SECOND: return 15.;
      case rm_msgs::ShootCmd::SPEED_16M_PER_SECOND: return 16.;
      case rm_msgs::ShootCmd::SPEED_18M_PER_SECOND: return 18.;
      case rm_msgs::ShootCmd::SPEED_30M_PER_SECOND: return 30.;
    }
    return 0.;
  }
  void setBurstMode(bool burst_flag) { heat_limit_->setMode(burst_flag); }
  bool getBurstMode() { return heat_limit_->getMode(); }
  void setZero() override {};
 private:
  double gimbal_error_limit_{};
  HeatLimit *heat_limit_{};
};

class Vel3DCommandSender : public HeaderStampCommandSenderBase<geometry_msgs::TwistStamped> {
 public:
  explicit Vel3DCommandSender(ros::NodeHandle &nh) : HeaderStampCommandSenderBase(nh) {
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
  void setLinearVel(double scale_x, double scale_y, double scale_z) {
    msg_.twist.linear.x = max_linear_x_ * scale_x;
    msg_.twist.linear.y = max_linear_y_ * scale_y;
    msg_.twist.linear.z = max_linear_z_ * scale_z;
  }
  void setAngularVel(double scale_x, double scale_y, double scale_z) {
    msg_.twist.angular.x = max_angular_x_ * scale_x;
    msg_.twist.angular.y = max_angular_y_ * scale_y;
    msg_.twist.angular.z = max_angular_z_ * scale_z;
  }
  void setZero() override {
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

class JointPositionBinaryCommandSender : public CommandSenderBase<std_msgs::Float64> {
 public:
  explicit JointPositionBinaryCommandSender(ros::NodeHandle &nh) :
      CommandSenderBase<std_msgs::Float64>(nh) {
    ROS_ASSERT(nh.getParam("close_pos", close_pos_) && nh.getParam("open_pos", open_pos_));
  }
  void open() {
    msg_.data = open_pos_;
    state = false;
  }
  void close() {
    msg_.data = close_pos_;
    state = true;
  }
  bool getState() const { return state; }
  void sendCommand(const ros::Time &time) override { CommandSenderBase<std_msgs::Float64>::sendCommand(time); }
  void setZero() override {};
 private:
  bool state{};
  double close_pos_{}, open_pos_{};
};

class JointJogCommandSender : public CommandSenderBase<std_msgs::Float64> {
 public:
  explicit JointJogCommandSender(ros::NodeHandle &nh, const sensor_msgs::JointState &joint_state) :
      CommandSenderBase<std_msgs::Float64>(nh), joint_state_(joint_state) {
    ROS_ASSERT(nh.getParam("joint", joint_));
    ROS_ASSERT(nh.getParam("step", step_));
  }
  void reset() {
    auto i = std::find(joint_state_.name.begin(), joint_state_.name.end(), joint_);
    if (i != joint_state_.name.end())
      msg_.data = joint_state_.position[std::distance(joint_state_.name.begin(), i)];
    else
      msg_.data = NAN;
  }
  void plus() {
    if (msg_.data != NAN) {
      msg_.data += step_;
      sendCommand(ros::Time());
    }
  }
  void minus() {
    if (msg_.data != NAN) {
      msg_.data -= step_;
      sendCommand(ros::Time());
    }
  }
  const std::string &getJoint() { return joint_; }
 private:
  std::string joint_{};
  const sensor_msgs::JointState &joint_state_;
  double step_{};
};
}

#endif // RM_COMMON_COMMAND_SENDER_H_
