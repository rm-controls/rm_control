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
  void input(double in);
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

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> realtime_pub_{};
};

class Filter {
 public:
  Filter() = default;
  virtual ~Filter() = default;
  virtual void input(double input_value) = 0;
  virtual double output() = 0;
  virtual void clear() = 0;
 protected:
  bool is_debug_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> realtime_pub_{};
};

class DigitalLpFilter : public Filter {
 public:
  explicit DigitalLpFilter(ros::NodeHandle &nh);
  ~DigitalLpFilter();
  void input(double input_value);
  double output();
  void clear();

 private:
  double Lpf_in_prev_[2];
  double Lpf_out_prev_[2];
  double Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
  double lpf_out_;
  double wc_{}, ts_{};
};

#endif //SRC_RM_SOFTWARE_RM_COMMON_INCLUDE_LP_FILTER_H_
