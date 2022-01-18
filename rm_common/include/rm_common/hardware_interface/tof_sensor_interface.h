//
// Created by luotinkai on 2021/12/31.
//

#pragma once

#include <utility>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class TofSensorHandle
{
public:
  TofSensorHandle() = default;
  TofSensorHandle(std::string name, double* distance, int* dis_status, double* signal_strength)
    : name_(std::move(name)), distance_(distance), dis_status_(dis_status), signal_strength_(signal_strength)
  {
    if (!distance_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. distance_ pointer is null.");
    if (!dis_status_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. dis_status_ pointer is null.");
    if (!signal_strength_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. signal_strength_ pointer is null.");
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
  double getStatus() const
  {
    assert(dis_status_);
    return *dis_status_;
  }
  double getStrength() const
  {
    assert(signal_strength_);
    return *signal_strength_;
  }

private:
  std::string name_;
  double* distance_;
  int* dis_status_;
  double* signal_strength_;
};

class TofSensorInterface
  : public hardware_interface::HardwareResourceManager<TofSensorHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace rm_control
