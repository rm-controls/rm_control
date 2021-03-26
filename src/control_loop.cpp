//
// Created by qiayuan on 12/30/20.
//
#include "rm_base/control_loop.h"

rm_base::RmBaseLoop::RmBaseLoop(ros::NodeHandle &nh, std::shared_ptr<RmBaseHardWareInterface> hardware_interface)
    : nh_(nh), hardware_interface_(std::move(hardware_interface)) {
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

  // Load rosparams
  int error = 0;
  ros::NodeHandle nh_p("~");
  error += !nh_p.getParam("loop_frequency", loop_hz_);
  error += !nh_p.getParam("cycle_time_error_threshold", cycle_time_error_threshold_);
  if (error > 0) {
    char error_message[] =
        "could not retrieve one of the required parameters\n\trm_base/loop_hz or rm_base/cycle_time_error_threshold";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  last_time_ = steady_clock::now();

  // Start timer that will periodically call RmBaseLoop::update
  desired_update_freq_ = ros::Duration(1 / loop_hz_);
  loop_timer_ = nh_.createTimer(desired_update_freq_, &RmBaseLoop::update, this);
}

void rm_base::RmBaseLoop::update(const ros::TimerEvent &) {

  // Get change in time
  current_time_ = steady_clock::now();
  duration<double> time_span
      = duration_cast<duration<double >>(current_time_ - last_time_);
  elapsed_time_ = ros::Duration(time_span.count());
  last_time_ = current_time_;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: "
                        << cycle_time_error - cycle_time_error_threshold_ << "s, "
                        << "cycle time: " << elapsed_time_ << "s, "
                        << "threshold: " << cycle_time_error_threshold_ << "s");
  }

  // Input
  // get the hardware's state
  hardware_interface_->read(ros::Time::now(), elapsed_time_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  // send the new command to hardware
  hardware_interface_->write(ros::Time::now(), elapsed_time_);

}


