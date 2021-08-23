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
// Created by qiayuan on 1/2/21.
//
#include "rm_hw/transmission/double_actuator_transmission_loader.h"
#include "rm_hw/transmission/double_actuator_transmission.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/internal/demangle_symbol.h>

namespace transmission_interface
{
TransmissionSharedPtr DoubleActuatorTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  // Transmission should contain only two actuator one joint
  if (!checkActuatorDimension(transmission_info, 2))
  {
    return TransmissionSharedPtr();
  }
  if (!checkJointDimension(transmission_info, 1))
  {
    return TransmissionSharedPtr();
  }

  // Get actuator and joint configuration sorted by role: [actuator1, actuator2] and [joint1]
  std::vector<double> act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok)
  {
    return TransmissionSharedPtr();
  }

  double jnt_reduction, jnt_offset;
  getJointConfig(transmission_info, jnt_reduction, jnt_offset);

  // Transmission instance
  try
  {
    TransmissionSharedPtr transmission(new DoubleActuatorTransmission(act_reduction, jnt_reduction, jnt_offset));
    return transmission;
  }
  catch (const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '"
                                         << transmission_info.name_ << "' of type '"
                                         << demangledTypeName<DoubleActuatorTransmissionLoader>() << "'. "
                                         << ex.what());
    return TransmissionSharedPtr();
  }
}

bool DoubleActuatorTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                         std::vector<double>& actuator_reduction)
{
  const std::string actuato_r1_role = "actuator1";
  const std::string actuato_r2_role = "actuator2";

  std::vector<TiXmlElement> act_elements(2, "");
  std::vector<std::string> act_names(2);
  std::vector<std::string> act_roles(2);

  for (unsigned int i = 0; i < 2; ++i)
  {
    // Actuator name
    act_names[i] = transmission_info.actuators_[i].name_;

    // Actuator xml element
    act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

    // Populate role string
    std::string& act_role = act_roles[i];
    getActuatorRole(act_elements[i], act_names[i], transmission_info.name_, true,  // Required
                    act_role);
    //    if (act_role_status != true) { return false; }

    // Validate role string
    if (actuato_r1_role != act_role && actuato_r2_role != act_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << act_names[i] << "' of transmission '" << transmission_info.name_
                                                    << "' does not specify a valid <role> element. Got '" << act_role
                                                    << "', expected '" << actuato_r1_role << "' or '" << actuato_r2_role
                                                    << "'.");
      return false;
    }
  }

  // Roles must be different
  if (act_roles[0] == act_roles[1])
  {
    ROS_ERROR_STREAM_NAMED("parser", "Actuators '" << act_names[0] << "' and '" << act_names[1] << "' of transmission '"
                                                   << transmission_info.name_
                                                   << "' must have different roles. Both specify '" << act_roles[0]
                                                   << "'.");
    return false;
  }

  // Indices sorted according to role
  std::vector<unsigned int> id_map(2);
  if (actuato_r1_role == act_roles[0])
  {
    id_map[0] = 0;
    id_map[1] = 1;
  }
  else
  {
    id_map[0] = 1;
    id_map[1] = 0;
  }

  // Parse required mechanical reductions
  actuator_reduction.resize(2);
  for (unsigned int i = 0; i < 2; ++i)
  {
    const unsigned int id = id_map[i];
    getActuatorReduction(act_elements[id], act_names[id], transmission_info.name_, true, actuator_reduction[i]);
  }

  return true;
}

bool DoubleActuatorTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                      double& joint_reduction, double& joint_offset)
{
  TiXmlElement jnt_elements = "";

  std::string jnt_names = transmission_info.joints_[0].name_;

  // Joint xml element
  jnt_elements = loadXmlElement(transmission_info.joints_[0].xml_element_);

  // Joint configuration
  // Parse optional mechanical reductions.
  getJointReduction(jnt_elements, jnt_names, transmission_info.name_, false, joint_reduction);
  // Parse optional joint offset.
  getJointOffset(jnt_elements, jnt_names, transmission_info.name_, false, joint_offset);
  return true;
}

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::DoubleActuatorTransmissionLoader,
                       transmission_interface::TransmissionLoader)
