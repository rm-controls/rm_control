//
// Created by qiayuan on 1/2/21.
//
#include "rm_base/transmission/double_actuator_transmission_loader.h"
#include "rm_base/transmission/double_actuator_transmission.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/internal/demangle_symbol.h>

namespace transmission_interface {

TransmissionSharedPtr DoubleActuatorTransmissionLoader::load(const TransmissionInfo &transmission_info) {
  // Transmission should contain only two actuator one joint
  if (!checkActuatorDimension(transmission_info, 2)) { return TransmissionSharedPtr(); }
  if (!checkJointDimension(transmission_info, 1)) { return TransmissionSharedPtr(); }

  // Get actuator and joint configuration sorted by role: [actuator1, actuator2] and [joint1]
  std::vector<double> act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok) { return TransmissionSharedPtr(); }

  double jnt_reduction, jnt_offset;
  const bool jnt_config_ok = getJointConfig(transmission_info, jnt_reduction, jnt_offset);

  if (!jnt_config_ok) { return TransmissionSharedPtr(); }

  // Transmission instance
  try {
    TransmissionSharedPtr transmission(new DoubleActuatorTransmission(act_reduction, jnt_reduction, jnt_offset));
    return transmission;
  }
  catch (const TransmissionInterfaceException &ex) {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser",
                           "Failed to construct transmission '" << transmission_info.name_ << "' of type '" <<
                                                                demangledTypeName<DoubleActuatorTransmissionLoader>()
                                                                << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

bool DoubleActuatorTransmissionLoader::getActuatorConfig(const TransmissionInfo &transmission_info,
                                                         std::vector<double> &actuator_reduction) {
  const std::string ACTUATOR1_ROLE = "actuator1";
  const std::string ACTUATOR2_ROLE = "actuator2";

  std::vector<TiXmlElement> act_elements(2, "");
  std::vector<std::string> act_names(2);
  std::vector<std::string> act_roles(2);

  for (unsigned int i = 0; i < 2; ++i) {
    // Actuator name
    act_names[i] = transmission_info.actuators_[i].name_;

    // Actuator xml element
    act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

    // Populate role string
    std::string &act_role = act_roles[i];
    getActuatorRole(act_elements[i], act_names[i],
                                           transmission_info.name_, true, // Required
                                           act_role);
//    if (act_role_status != true) { return false; }

    // Validate role string
    if (ACTUATOR1_ROLE != act_role && ACTUATOR2_ROLE != act_role) {
      ROS_ERROR_STREAM_NAMED("parser",
                             "Actuator '" << act_names[i] << "' of transmission '" << transmission_info.name_ <<
                                          "' does not specify a valid <role> element. Got '" << act_role
                                          << "', expected '" <<
                                          ACTUATOR1_ROLE << "' or '" << ACTUATOR2_ROLE << "'.");
      return false;
    }
  }

  // Roles must be different
  if (act_roles[0] == act_roles[1]) {
    ROS_ERROR_STREAM_NAMED("parser",
                           "Actuators '" << act_names[0] << "' and '" << act_names[1] <<
                                         "' of transmission '" << transmission_info.name_ <<
                                         "' must have different roles. Both specify '" << act_roles[0] << "'.");
    return false;
  }

  // Indices sorted according to role
  std::vector<unsigned int> id_map(2);
  if (ACTUATOR1_ROLE == act_roles[0]) {
    id_map[0] = 0;
    id_map[1] = 1;
  } else {
    id_map[0] = 1;
    id_map[1] = 0;
  }

  // Parse required mechanical reductions
  actuator_reduction.resize(2);
  for (unsigned int i = 0; i < 2; ++i) {
    const unsigned int id = id_map[i];
    getActuatorReduction(act_elements[id], act_names[id], transmission_info.name_,
                                                 true, actuator_reduction[i]);
  }

  return true;
}

bool DoubleActuatorTransmissionLoader::getJointConfig(const TransmissionInfo &transmission_info,
                                                      double &joint_reduction, double &joint_offset) {
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

}

PLUGINLIB_EXPORT_CLASS(transmission_interface::DoubleActuatorTransmissionLoader,
                       transmission_interface::TransmissionLoader)