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

  TfRadarHandle(std::string name, double* distance, int* strength, double* signal)
    : name_(std::move(name)), distance_(distance), strength_(strength), signal_(signal)
  {
    if (!distance_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. distance_ pointer is null.");
    if (!strength_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. strength_ pointer is null.");
    if (!signal_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. signal_ pointer is null.");
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

  double getSignal() const
  {
    assert(signal_);
    return *signal_;
  }

private:
  std::string name_;
  double* distance_;
  int* strength_;
  double* signal_;
};

class TfRadarInterface
  : public hardware_interface::HardwareResourceManager<TfRadarHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace rm_control
