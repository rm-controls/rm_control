//
// Created by luotinkai on 2021/12/5.
//

#pragma once

#include <tinyxml.h>
#include <transmission_interface/transmission_loader.h>
#include "rm_hw/transmission/multi_actuator_transmission.h"

namespace transmission_interface
{
class MultiActuatorTransmissionLoader : public TransmissionLoader
{
public:
  TransmissionSharedPtr load(const TransmissionInfo& transmission_info) override;

private:
  static bool getActuatorConfig(const TransmissionInfo& transmission_info, std::vector<double>& actuator_reduction);
  static bool getJointConfig(const TransmissionInfo& transmission_info, double& joint_reduction, double& joint_offset);
};

}  // namespace transmission_interface
