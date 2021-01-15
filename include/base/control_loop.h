//
// Created by qiayuan on 12/30/20.
//

#ifndef RM_BASE_INCLUDE_BASE_CONTROL_LOOP_H_
#define RM_BASE_INCLUDE_BASE_CONTROL_LOOP_H_

#include "base/hardware_interface/hardware_interface.h"

// Timer
#include <chrono>
#include <utility>

// ROS
#include <ros/ros.h>

// ROS control
#include <controller_manager/controller_manager.h>

namespace rm_base {
using namespace std::chrono;

class RmBaseLoop {
 public:
  RmBaseLoop(ros::NodeHandle &nh,
             std::shared_ptr<RmBaseHardWareInterface> hardware_interface);

  /** Timed method that reads current hardware's state, runs the controller
      code once and sends the new commands to the hardware.

      Note: we do not use the TimerEvent time difference because it
          does NOT guarantee that the time source is strictly
          linearly increasing.
  **/
  void update(const ros::TimerEvent &);

 private:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Settings
  ros::Duration desired_update_freq_;
  double cycle_time_error_threshold_{};

  // Timing
  ros::Timer loop_timer_;
  ros::Duration elapsed_time_;
  double loop_hz_{};
  steady_clock::time_point last_time_;
  steady_clock::time_point current_time_;

  /** ROS Controller Manager and Runner

      This class advertises a ROS interface for loading, unloading, starting, and
      stopping ros_control-based controllers. It also serializes execution of all
      running controllers in \ref update.
  **/
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<RmBaseHardWareInterface> hardware_interface_;
};
}// namespace rm_base

#endif //RM_BASE_INCLUDE_BASE_CONTROL_LOOP_H_
