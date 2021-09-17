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

#include "rm_hw/hardware_interface/hardware_interface.h"

#include <rm_common/ros_utilities.h>

namespace rm_hw
{
bool RmRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  // Parse actuator coefficient specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("actuator_coefficient", xml_rpc_value))
    ROS_WARN("No actuator coefficient specified");
  else if (!parseActCoeffs(xml_rpc_value))
    return false;
  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("actuators", xml_rpc_value))
    ROS_WARN("No actuator specified");
  else if (!parseActData(xml_rpc_value, robot_hw_nh))
    return false;
  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else if (!parseImuData(xml_rpc_value, robot_hw_nh))
    return false;
  // CAN Bus
  if (!robot_hw_nh.getParam("bus", xml_rpc_value))
    ROS_WARN("No bus specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    for (int i = 0; i < xml_rpc_value.size(); ++i)
    {
      std::string bus_name = xml_rpc_value[i];
      if (bus_name.find("can") != std::string::npos)
        can_buses_.push_back(new CanBus(bus_name, CanDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                              .id2act_data_ = &bus_id2act_data_[bus_name],
                                                              .id2imu_data_ = &bus_id2imu_data_[bus_name] }));
      else
        ROS_ERROR_STREAM("Unknown bus: " << bus_name);
    }
  }

  if (!loadUrdf(root_nh))
  {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }
  // Initialize transmission
  if (!setupTransmission(root_nh))
  {
    ROS_ERROR("Error occurred while setting up transmission");
    return false;
  }
  // Initialize joint limit
  if (!setupJointLimit(root_nh))
  {
    ROS_ERROR("Error occurred while setting up joint limit");
    return false;
  }

  // Other Interface
  registerInterface(&robot_state_interface_);

  actuator_state_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::ActuatorState>(root_nh, "/actuator_states", 100));
  return true;
}

void RmRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  for (auto bus : can_buses_)
    bus->read(time);
  for (auto& id2act_datas : bus_id2act_data_)
    for (auto& act_data : id2act_datas.second)
    {
      try
      {  // Duration will be out of dual 32-bit range while motor failure
        act_data.second.halted = (time - act_data.second.stamp).toSec() > 0.1 || act_data.second.temp > 99;
      }
      catch (std::runtime_error& ex)
      {
      }
      if (act_data.second.halted)
      {
        act_data.second.seq = 0;
        act_data.second.vel = 0;
        act_data.second.effort = 0;
        act_data.second.calibrated = false;  // set the actuator no calibrated
      }
    }
  if (is_actuator_specified_)
    act_to_jnt_state_->propagate();
  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto effort_joint_handle : effort_joint_handles_)
    effort_joint_handle.setCommand(0.);
}

void RmRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  if (is_actuator_specified_)
  {
    // Propagate without joint limits
    jnt_to_act_effort_->propagate();
    // Save commanded effort before enforceLimits
    for (auto& id2act_datas : bus_id2act_data_)
      for (auto& act_data : id2act_datas.second)
        act_data.second.cmd_effort = act_data.second.exe_effort;
    // enforceLimits will limit cmd_effort into suitable value https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
    effort_jnt_saturation_interface_.enforceLimits(period);
    effort_jnt_soft_limits_interface_.enforceLimits(period);
    // Propagate with joint limits
    jnt_to_act_effort_->propagate();
    // Restore the cmd_effort for the calibrating joint
    for (auto& id2act_datas : bus_id2act_data_)
      for (auto& act_data : id2act_datas.second)
        if (act_data.second.need_calibration && !act_data.second.calibrated)
          act_data.second.exe_effort = act_data.second.cmd_effort;
  }
  for (auto& bus : can_buses_)
    bus->write();
  publishActuatorState(time);
}

void RmRobotHW::publishActuatorState(const ros::Time& time)
{
  if (!is_actuator_specified_)
    return;
  if (last_publish_time_ + ros::Duration(1.0 / 100.0) < time)
  {
    if (actuator_state_pub_->trylock())
    {
      rm_msgs::ActuatorState actuator_state;
      for (const auto& id2act_datas : bus_id2act_data_)
        for (const auto& act_data : id2act_datas.second)
        {
          actuator_state.stamp.push_back(act_data.second.stamp);
          actuator_state.name.push_back(act_data.second.name);
          actuator_state.type.push_back(act_data.second.type);
          actuator_state.bus.push_back(id2act_datas.first);
          actuator_state.id.push_back(act_data.first);
          actuator_state.halted.push_back(act_data.second.halted);
          actuator_state.need_calibration.push_back(act_data.second.need_calibration);
          actuator_state.calibrated.push_back(act_data.second.calibrated);
          actuator_state.calibration_reading.push_back(act_data.second.calibration_reading);
          actuator_state.position_raw.push_back(act_data.second.q_raw);
          actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
          actuator_state.temperature.push_back(act_data.second.temp);
          actuator_state.circle.push_back(act_data.second.q_circle);
          actuator_state.last_position_raw.push_back(act_data.second.q_last);
          actuator_state.frequency.push_back(act_data.second.frequency);
          actuator_state.position.push_back(act_data.second.pos);
          actuator_state.velocity.push_back(act_data.second.vel);
          actuator_state.effort.push_back(act_data.second.effort);
          actuator_state.commanded_effort.push_back(act_data.second.cmd_effort);
          actuator_state.executed_effort.push_back(act_data.second.exe_effort);
          actuator_state.offset.push_back(act_data.second.offset);
        }
      actuator_state_pub_->msg_ = actuator_state;
      actuator_state_pub_->unlockAndPublish();
      last_publish_time_ = time;
    }
  }
}
}  // namespace rm_hw
