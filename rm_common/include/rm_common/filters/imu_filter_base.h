//
// Created by yezi on 2022/3/26.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <realtime_tools/realtime_publisher.h>

namespace rm_common
{
class ImuFilterBase
{
public:
  bool init(XmlRpc::XmlRpcValue& imu_data, const std::string& name);
  void update(ros::Time time, double* accel, double* omega, double* ori, double* accel_cov, double* omega_cov,
              double* ori_cov, double temp, bool camera_trigger);
  virtual void getOrientation(double& q0, double& q1, double& q2, double& q3) = 0;

protected:
  virtual bool getFilterParam(XmlRpc::XmlRpcValue& imu_data) = 0;
  virtual void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) = 0;
  ros::Time last_update_;
  bool initialized_filter_{ false };
  std::string frame_id_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > imu_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Temperature> > imu_temp_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::TimeReference> > trigger_time_pub_;
};
}  // namespace rm_common
