//
// Created by yezi on 2022/3/26.
//

#include "rm_common/filters/imu_filter_base.h"

namespace rm_common
{
bool ImuFilterBase::init(XmlRpc::XmlRpcValue& imu_data, const std::string& name)
{
  ros::NodeHandle nh("~");
  frame_id_ = (std::string)imu_data["frame_id"];
  initFilter(imu_data);
  imu_data_pub_ = nh.advertise<sensor_msgs::Imu>(name + "/data", 100);
  imu_temp_pub_ = nh.advertise<sensor_msgs::Temperature>(name + "/temperature", 100);
  trigger_time_pub_ = nh.advertise<sensor_msgs::TimeReference>(name + "/trigger_time", 100);
  return true;
}

void ImuFilterBase::update(ros::Time time, double* accel, double* omega, double* ori, double* accel_cov,
                           double* omega_cov, double* ori_cov, double temp, bool camera_trigger)
{
  if (!initialized_filter_)
  {
    initialized_filter_ = true;
    last_update_ = time;
    imu_pub_data_.header.frame_id = frame_id_;
    return;
  }
  if ((time - last_update_).toSec() > 1)
  {
    resetFilter();
    last_update_ = time;
    return;
  }
  // Update the filter.
  filterUpdate(accel[0], accel[1], accel[2], omega[0], omega[1], omega[2], (time - last_update_).toSec());
  last_update_ = time;
  getOrientation(ori[3], ori[0], ori[1], ori[2]);
  if (camera_trigger)
  {
    imu_pub_data_.header.stamp = time;
    imu_pub_data_.angular_velocity.x = omega[0];
    imu_pub_data_.angular_velocity.y = omega[1];
    imu_pub_data_.angular_velocity.z = omega[2];
    imu_pub_data_.linear_acceleration.x = accel[0];
    imu_pub_data_.linear_acceleration.y = accel[1];
    imu_pub_data_.linear_acceleration.z = accel[2];
    imu_pub_data_.orientation.x = ori[0];
    imu_pub_data_.orientation.y = ori[1];
    imu_pub_data_.orientation.z = ori[2];
    imu_pub_data_.orientation.w = ori[3];
    imu_pub_data_.orientation_covariance = { ori_cov[0], 0., 0., 0., ori_cov[4], 0., 0., 0., ori_cov[8] };
    imu_pub_data_.angular_velocity_covariance = { omega_cov[0], 0., 0., 0., omega_cov[4], 0., 0., 0., omega_cov[8] };
    imu_pub_data_.linear_acceleration_covariance = { accel_cov[0], 0., 0., 0., accel_cov[4], 0., 0., 0., accel_cov[8] };
    imu_data_pub_.publish(imu_pub_data_);

    trigger_time_pub_data_.header.stamp = time;
    trigger_time_pub_data_.time_ref = time;
    trigger_time_pub_.publish(trigger_time_pub_data_);

    temp_pub_data_.header.stamp = time;
    temp_pub_data_.temperature = temp;
    imu_temp_pub_.publish(temp_pub_data_);
  }
}

}  // namespace rm_common
