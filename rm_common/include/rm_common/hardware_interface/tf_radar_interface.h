//
// Created by luotinkai on 2022/4/17.
//

#pragma once

#include <utility>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class TfRadarHandle
{
public:
  TfRadarHandle() = default;

  TfRadarHandle(std::string name, double* distance, double* strength)
    : name_(std::move(name)), distance_(distance), strength_(strength)
  {
    if (!distance_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. distance_ pointer is null.");
    if (!strength_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. strength_ pointer is null.");
  }

  std::string getName() const
  {
    return name_;
  }

  double getDistance() const
  {
    assert(distance_);
    return *distance_;
  }

  double getStrength() const
  {
    assert(strength_);
    return *strength_;
  }

private:
  std::string name_;
  double* distance_;
  double* strength_;
};

class TfRadarInterface
  : public hardware_interface::HardwareResourceManager<TfRadarHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace rm_control
