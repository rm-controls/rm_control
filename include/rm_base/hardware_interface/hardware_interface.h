//
// Created by qiayuan on 12/21/20.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_HARDWARE_INTERFACE_H_
#define RM_BASE_INCLUDE_RM_BASE_HARDWARE_INTERFACE_H_

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <XmlRpcValue.h>

// ROS control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <rm_msgs/ActuatorState.h>

#include "can_bus.h"

namespace rm_base {

class RmBaseHardWareInterface : public hardware_interface::RobotHW {
 public:
  RmBaseHardWareInterface() = default;

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  void read(const ros::Time &time, const ros::Duration &period) override;

  void write(const ros::Time &time, const ros::Duration &period) override;

 private:
  bool is_actuator_specified_ = false;

  bool parseActCoeffs(XmlRpc::XmlRpcValue &act_coeffs);
  bool parseActData(XmlRpc::XmlRpcValue &act_datas, ros::NodeHandle &robot_hw_nh);
  bool parseImuData(XmlRpc::XmlRpcValue &imu_datas, ros::NodeHandle &robot_hw_nh);
  bool load_urdf(ros::NodeHandle &root_nh);
  bool setupTransmission(ros::NodeHandle &root_nh);
  bool setupJointLimit(ros::NodeHandle &root_nh);
  void publishActuatorState(const ros::Time &time);

  // Interface
  std::vector<CanBus *> can_buses_{};
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::ActuatorExtraInterface act_extra_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  hardware_interface::RobotStateInterface robot_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface *act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface *jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};

  // URDF model of the robot
  std::string urdf_string_;                   // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;   // for limit

  // Actuator
  std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // Imu
  std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};

  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ActuatorState>> actuator_state_pub_;

};

}
#endif //RM_BASE_INCLUDE_RM_BASE_HARDWARE_INTERFACE_H_
