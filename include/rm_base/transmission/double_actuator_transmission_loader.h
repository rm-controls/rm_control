//
// Created by qiayuan on 1/2/21.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_LOADER_H_
#define RM_BASE_INCLUDE_RM_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_LOADER_H_

#pragma once

#include <tinyxml.h>
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface {

class DoubleActuatorTransmissionLoader : public TransmissionLoader {
 public:
  TransmissionSharedPtr load(const TransmissionInfo &transmission_info) override;

 private:
  static bool getActuatorConfig(const TransmissionInfo &transmission_info, std::vector<double> &actuator_reduction);
  static bool getJointConfig(const TransmissionInfo &transmission_info, double &joint_reduction, double &joint_offset);
};

}
#endif //RM_BASE_INCLUDE_RM_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_LOADER_H_
