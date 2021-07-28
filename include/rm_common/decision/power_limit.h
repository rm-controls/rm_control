//
// Created by bruce on 2021/7/28.
//

#ifndef RM_COMMON_POWER_LIMIT_H_
#define RM_COMMON_POWER_LIMIT_H_
#include <ros/ros.h>
#include <rm_msgs/ChassisCmd.h>

#include "rm_common/referee/data.h"
namespace rm_common {
class PowerLimit {
 public:
  PowerLimit(ros::NodeHandle &nh, const RefereeData &referee_data, const rm_msgs::ChassisCmd &chassis_cmd)
      : referee_data_(referee_data), chassis_cmd_(chassis_cmd) {
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
  }
  typedef enum {
    TEST = 0,
    BURST = 1,
    NORMAL = 2,
    CHARGE = 3,
  } Mode;

  void updateState(uint8_t state) { state_ = state; }
  uint8_t getState() { return state_; }
  double getLimitPower() {
    if (referee_data_.robot_id_ == rm_common::RobotId::BLUE_SENTRY
        || referee_data_.robot_id_ == rm_common::RobotId::RED_SENTRY)
      limit_power_ = 30;
    else if (referee_data_.robot_id_ == rm_common::RobotId::RED_ENGINEER
        || referee_data_.robot_id_ == rm_common::RobotId::BLUE_ENGINEER)
      limit_power_ = 300;
    else {//standard and hero
      if (referee_data_.is_online_) {
        if (referee_data_.capacity_data.is_online_) {
          if (referee_data_.capacity_data.limit_power_ == 0 && referee_data_.is_online_)
            return 30;//calibra
          if (referee_data_.game_robot_status_.chassis_power_limit_ > 120)
            limit_power_ = burst_power_;
          else {
            switch (state_) {
              case TEST: test();
                break;
              case BURST: burst();
                break;
              case NORMAL: normal();
                break;
              case CHARGE: charge();
                break;
            }
            if (!(state_ == Mode::BURST) && (abs(
                referee_data_.capacity_data.limit_power_ - referee_data_.game_robot_status_.chassis_power_limit_)
                < 0.05))
              normal();
          }
        } else
          limit_power_ = referee_data_.game_robot_status_.chassis_power_limit_;
      } else
        limit_power_ = safety_power_;
    }
    return limit_power_;
  }

 private:
  void charge() { limit_power_ = referee_data_.game_robot_status_.chassis_power_limit_ * 0.85; }
  void normal() { limit_power_ = referee_data_.game_robot_status_.chassis_power_limit_; }
  void test() { limit_power_ = 0.0; }
  void burst() {
    if (referee_data_.capacity_data.cap_power_ > capacitor_threshold_) {
      if (chassis_cmd_.mode == rm_msgs::ChassisCmd::GYRO)
        limit_power_ = referee_data_.game_robot_status_.chassis_power_limit_ + extra_power_;
      else
        limit_power_ = burst_power_;
    } else
      limit_power_ = referee_data_.game_robot_status_.chassis_power_limit_;
  }

  double limit_power_;
  double safety_power_{};
  double capacitor_threshold_{};
  double charge_power_{};
  double extra_power_{};
  double burst_power_{};
  uint8_t state_{};
  const RefereeData &referee_data_;
  const rm_msgs::ChassisCmd &chassis_cmd_;
};
}

#endif //SRM_COMMON_POWER_LIMIT_H_
