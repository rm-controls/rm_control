//
// Created by yezi on 2022/3/26.
//

#include "rm_common/filters/imu_complementary_filter.h"

namespace rm_common
{
void ImuComplementaryFilter::filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt)
{
  filter_->update(ax, ay, az, wx, wy, wz, dt);
}
void ImuComplementaryFilter::getOrientation(double& q0, double& q1, double& q2, double& q3)
{
  filter_->getOrientation(q0, q1, q2, q3);
}
bool ImuComplementaryFilter::initFilter(XmlRpc::XmlRpcValue& imu_data)
{
  use_mag_ = imu_data.hasMember("use_mag") && (bool)imu_data["use_mag"];
  gain_acc_ = imu_data.hasMember("gain_acc") ? (double)imu_data["gain_acc"] : 0.01;
  gain_mag_ = imu_data.hasMember("gain_mag") ? (double)imu_data["gain_mag"] : 0.01;
  do_bias_estimation_ = !imu_data.hasMember("do_bias_estimation") || (bool)imu_data["do_bias_estimation"];
  bias_alpha_ = imu_data.hasMember("bias_alpha") ? (double)imu_data["bias_alpha"] : 0.01;
  do_adaptive_gain_ = !imu_data.hasMember("do_adaptive_gain") || (bool)imu_data["do_adaptive_gain"];
  resetFilter();
  return true;
}
void ImuComplementaryFilter::resetFilter()
{
  filter_ = std::make_shared<imu_tools::ComplementaryFilter>();
  filter_->setDoBiasEstimation(do_bias_estimation_);
  filter_->setDoAdaptiveGain(do_adaptive_gain_);
  if (!filter_->setGainAcc(gain_acc_))
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if (!filter_->setGainMag(gain_mag_))
      ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation_)
  {
    if (!filter_->setBiasAlpha(bias_alpha_))
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
  }
  return;
}
}  // namespace rm_common
