//
// Created by luotinkai on 2021/12/5.
//

#include "rm_hw/transmission/multi_actuator_transmission_loader.h"
#include "rm_hw/transmission/multi_actuator_transmission.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/internal/demangle_symbol.h>

namespace transmission_interface
{
TransmissionSharedPtr MultiActuatorTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  std::vector<double> act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok)
  {
    return TransmissionSharedPtr();
  }

  double jnt_reduction, jnt_offset;
  getJointConfig(transmission_info, jnt_reduction, jnt_offset);

  try
  {
    TransmissionSharedPtr transmission(
        new MultiActuatorTransmission(transmission_info, act_reduction, jnt_reduction, jnt_offset));
    return transmission;
  }
  catch (const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '"
                                         << transmission_info.name_ << "' of type '"
                                         << demangledTypeName<MultiActuatorTransmissionLoader>() << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

bool MultiActuatorTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                        std::vector<double>& actuator_reduction)
{
  const std::string actuator_role = "main";
  static unsigned int act_num = transmission_info.actuators_.size();
  std::vector<TiXmlElement> act_elements(act_num, "");
  std::vector<std::string> act_names(act_num);
  std::vector<std::string> act_roles(act_num);

  for (unsigned int i = 0; i < act_num; ++i)
  {
    act_names[i] = transmission_info.actuators_[i].name_;

    act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

    std::string& act_role = act_roles[i];
    getActuatorRole(act_elements[i], act_names[i], transmission_info.name_, true,  // Required
                    act_role);
  }
  if (!(actuator_role == act_roles[0]))
  {
    ROS_ERROR_STREAM_NAMED("parser", "Could not find main actuator");
    return false;
  }
  actuator_reduction.resize(act_num);
  for (unsigned int i = 0; i < act_num; ++i)
  {
    getActuatorReduction(act_elements[i], act_names[i], transmission_info.name_, true, actuator_reduction[i]);
  }

  return true;
}

bool MultiActuatorTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info, double& joint_reduction,
                                                     double& joint_offset)
{
  TiXmlElement jnt_elements = "";

  std::string jnt_names = transmission_info.joints_[0].name_;

  jnt_elements = loadXmlElement(transmission_info.joints_[0].xml_element_);

  getJointReduction(jnt_elements, jnt_names, transmission_info.name_, false, joint_reduction);
  getJointOffset(jnt_elements, jnt_names, transmission_info.name_, false, joint_offset);
  return true;
}

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::MultiActuatorTransmissionLoader,
                       transmission_interface::TransmissionLoader)
