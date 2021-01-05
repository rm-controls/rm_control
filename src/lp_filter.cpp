//
// Created by qiayuan on 1/5/21.
//

#include "lp_filter.h"
#include "ros_utilities.h"

LowPassFilter::LowPassFilter(ros::NodeHandle &nh, double cutoff_frequency) {
  nh.param("cutoff_frequency", cutoff_frequency_, -1.);
  nh.param("lp_filter_debug", is_debug_, false);

  if (cutoff_frequency > 0.)
    cutoff_frequency_ = cutoff_frequency;

  if (is_debug_) {
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(nh, "/lp_filter", 100));
  }
}

void LowPassFilter::input(double in, const ros::Duration &period) {
  // My filter reference was Julius O. Smith III, Intro. to Digital Filters
  // With Audio Applications.
  // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
  in_[2] = in_[1];
  in_[1] = in_[0];
  in_[0] = in;

  if (cutoff_frequency_ != -1) {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_ * 6.2832) * period.toSec() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;

    c_ = 1 / tan_filt_;
  }
  out_[2] = out_[1];
  out_[1] = out_[0];
  out_[0] = (1 / (1 + c_ * c_ + 1.414 * c_)) * (in_[0] + 2 * in_[1] + in_[1] -
      (c_ * c_ - 1.414 * c_ + 1) * out_[2] -
      (-2 * c_ * c_ + 2) * out_[1]);

  if (is_debug_) {
    if (realtime_pub_->trylock()) {
      realtime_pub_->msg_.data.push_back(in_[0]);
      realtime_pub_->msg_.data.push_back(out_[0]);
      realtime_pub_->unlockAndPublish();
    }
  }
}

double LowPassFilter::output() {
  return out_[0];
}
