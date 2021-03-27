//
// Created by qiayuan on 1/5/21.
//

#ifndef RM_COMMON_FILTERS_LP_FILTER_H
#define RM_COMMON_FILTERS_LP_FILTER_H
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/LpData.h>

class LowPassFilter {
 public:
  explicit LowPassFilter(ros::NodeHandle &nh);
  void input(double in);
  void input(double in, ros::Time time);
  double output();
 private:
  double in_[3]{};
  double out_[3]{};

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_ = -1;
  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at 1/4 of the sample rate.
  double c_ = 1.;
  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;
  bool is_debug_{};

  ros::Time prev_time_;
  ros::Duration delta_t_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LpData>> realtime_pub_{};

};

#endif //RM_COMMON_FILTERS_LP_FILTER_H
