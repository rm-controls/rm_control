/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/21/20.
//

#pragma once

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
#include <rm_common/hardware_interface/imu_extra_interface.h>
#include <rm_msgs/ActuatorState.h>

#include "can_bus.h"

namespace rm_hw
{
class RmRobotHW : public hardware_interface::RobotHW
{
public:
  RmRobotHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
   *
   * Call @ref rm_hw::CanBus::read(). Check whether temperature of actuator is too high and whether actuator is offline.
   * Propagate actuator state to joint state for the stored transmission. Set all cmd to zero to avoid crazy soft limit
   * oscillation when not controller loaded(all controllers update after read()).
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref rm_hw::CanBus::write(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  /** \brief Check whether some coefficients that are related to actuator are set up and load these coefficients.
   *
   * Check whether some coefficients that are related to actuator are set up and load these coefficients.
   *
   * @param act_coeffs Coefficients you want to check and load.
   * @return True if all coefficients are set up.
   */
  bool parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs);
  /** \brief Check whether actuator is specified and load specified params.
   *
   * Check whether actuator is specified and load specified params.
   *
   * @param act_datas Params you want to check and load.
   * @param robot_hw_nh Root node-handle of a ROS node.
   * @return True if all params are set up.
   */
  bool parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh);
  /** \brief Check whether some params that are related to imu are set up and load these params.
   *
   * Check whether some params that are related to imu are set up and load these params.
   *
   * @param imu_datas Params you want to check
   * @param robot_hw_nh Root node-handle of a ROS node
   * @return True if all params are set up.
   */
  bool parseImuData(XmlRpc::XmlRpcValue& imu_datas, ros::NodeHandle& robot_hw_nh);
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param root_nh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& root_nh);
  /** \brief Set up transmission.
   *
   * Set up transmission
   *
   * @param root_nh Root node-handle of a ROS node.
   * @return True if successful.
   */
  bool setupTransmission(ros::NodeHandle& root_nh);
  /** \brief Set up joint limit.
   *
   * Set up joint limit.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @return True if successful.
   */
  bool setupJointLimit(ros::NodeHandle& root_nh);
  /** \brief Publish actuator's state to a topic named "/actuator_states".
   *
   * Publish actuator's state to a topic named "/actuator_states".
   *
   * @param time Current time
   */
  void publishActuatorState(const ros::Time& time);

  bool is_actuator_specified_ = false;
  // Interface
  std::vector<CanBus*> can_buses_{};
  hardware_interface::ActuatorStateInterface act_state_interface_;
  rm_control::ActuatorExtraInterface act_extra_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  rm_control::RobotStateInterface robot_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  rm_control::ImuExtraInterface imu_extra_interface_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface* jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  // Actuator
  std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // Imu
  std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};

  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ActuatorState>> actuator_state_pub_;
};

}  // namespace rm_hw
