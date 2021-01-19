//
// Created by qiayuan on 1/5/21.
//

#ifndef RM_COMMON_INCLUDE_LP_FILTER_H_
#define RM_COMMON_INCLUDE_LP_FILTER_H_
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64MultiArray.h>

class LowPassFilter {
 public:
  explicit LowPassFilter(ros::NodeHandle &nh);
  void input(const ros::Time &time, const ros::Duration &period, double in);
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

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> realtime_pub_{};
};

#endif //SRC_RM_SOFTWARE_RM_COMMON_INCLUDE_LP_FILTER_H_
