//
// Created by yezi on 2021/9/22.
//

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
enum GpioType
{
  INPUT,
  OUTPUT
};

struct GpioData
{
  std::string name;
  GpioType type;
  int pin;
  bool* value;
};

class GpioStateHandle
{
public:
  GpioStateHandle() = default;
  GpioStateHandle(std::string name, GpioType type, bool* value) : name_(std::move(name)), type_(type), value_(value)
  {
    if (!value)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. value pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  GpioType getType() const
  {
    return type_;
  }
  bool getValue() const
  {
    assert(value_);
    return *value_;
  }

private:
  std::string name_;
  GpioType type_;
  bool* value_ = { nullptr };
};

class GpioCommandHandle
{
public:
  GpioCommandHandle() = default;
  GpioCommandHandle(std::string name, GpioType type, bool* cmd) : name_(std::move(name)), type_(type), cmd_(cmd)
  {
    if (!cmd)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. command pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  bool getCommand() const
  {
    assert(cmd_);
    return *cmd_;
  }

  void setCommand(bool value)
  {
    assert(cmd_);
    *cmd_ = value;
  }

private:
  std::string name_;
  GpioType type_;
  bool* cmd_ = { nullptr };
};

class GpioStateInterface
  : public hardware_interface::HardwareResourceManager<GpioStateHandle, hardware_interface::DontClaimResources>
{
};

class GpioCommandInterface
  : public hardware_interface::HardwareResourceManager<GpioCommandHandle, hardware_interface::ClaimResources>
{
};

}  // namespace rm_control
