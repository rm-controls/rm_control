//
// Created by yezi on 2021/9/22.
//

#ifndef SRC_RM_SOFTWARE_RM_CONTROL_RM_COMMON_INCLUDE_RM_COMMON_HARDWARE_INTERFACE_GPIO_STATE_INTERFACE_H_
#define SRC_RM_SOFTWARE_RM_CONTROL_RM_COMMON_INCLUDE_RM_COMMON_HARDWARE_INTERFACE_GPIO_STATE_INTERFACE_H_

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class GpioStateHandle
{
public:
  GpioStateHandle() = default;
  GpioStateHandle(std::string name, bool* value) : name_(std::move(name)), value_(value)
  {
    if (!value)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. value pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  int getValue() const
  {
    assert(value_);
    return *value_;
  }

private:
  std::string name_;
  bool* value_ = { nullptr };
};

class GpioStateInterface
  : public hardware_interface::HardwareResourceManager<GpioStateHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace rm_control

#endif  // SRC_RM_SOFTWARE_RM_CONTROL_RM_COMMON_INCLUDE_RM_COMMON_HARDWARE_INTERFACE_GPIO_STATE_INTERFACE_H_
