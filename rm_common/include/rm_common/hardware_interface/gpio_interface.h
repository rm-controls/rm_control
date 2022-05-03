//
// Created by yezi on 2021/9/22.
//

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class GpioReadHandle
{
public:
  GpioReadHandle() = default;
  GpioReadHandle(std::string name, bool* value) : name_(std::move(name)), value_(value)
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

class GpioWriteHandle
{
public:
  GpioWriteHandle() = default;
  GpioWriteHandle(std::string name, bool* cmd) : name_(std::move(name)), cmd_(cmd)
  {
    if (!cmd)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. command pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  int getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }

  void setCommand(bool value) const
  {
    assert(cmd_);
    *cmd_ = value;
  }

private:
  std::string name_;
  bool* cmd_ = { nullptr };
};

class GpioReadInterface
  : public hardware_interface::HardwareResourceManager<GpioReadHandle, hardware_interface::DontClaimResources>
{
};

class GpioWriteInterface
  : public hardware_interface::HardwareResourceManager<GpioWriteHandle, hardware_interface::ClaimResources>
{
};

}  // namespace rm_control
