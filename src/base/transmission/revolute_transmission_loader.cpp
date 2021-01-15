//
// Created by qiayuan on 1/2/21.
//

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

#include "base/transmission/revolute_transmission_loader.h"

namespace transmission_interface {

TransmissionSharedPtr RevoluteTransmissionLoader::load(const TransmissionInfo &transmission_info) {
  // Transmission should contain only one actuator/joint
  if (!checkActuatorDimension(transmission_info, 1)) { return TransmissionSharedPtr(); }
  if (!checkJointDimension(transmission_info, 1)) { return TransmissionSharedPtr(); }

  // Parse actuator and joint xml elements
  TiXmlElement actuator_el = loadXmlElement(transmission_info.actuators_.front().xml_element_);
  TiXmlElement joint_el = loadXmlElement(transmission_info.joints_.front().xml_element_);

  // Parse required mechanical reduction
  double reduction = 0.0;
  if (!getActuatorReduction
      (actuator_el, transmission_info.actuators_.front().name_, transmission_info.name_, true, // Required
       reduction)) { return TransmissionSharedPtr(); }

  // Parse optional joint offset. Even though it's optional --and to avoid surprises-- we fail if the element is
  // specified but is of the wrong type
  double joint_offset = 0.0;
  if (!getJointOffset
      (joint_el, transmission_info.joints_.front().name_, transmission_info.name_, false, // Optional
       joint_offset)) { return TransmissionSharedPtr(); }

  // Transmission instance
  try {
    TransmissionSharedPtr transmission(new RevoluteTransmission(reduction, joint_offset));
    return transmission;
  }
  catch (const TransmissionInterfaceException &ex) {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" <<
                                                                          transmission_info.name_ << "' of type '"
                                                                          << demangledTypeName<RevoluteTransmission>()
                                                                          << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}
}

PLUGINLIB_EXPORT_CLASS(transmission_interface::RevoluteTransmissionLoader,
                       transmission_interface::TransmissionLoader)