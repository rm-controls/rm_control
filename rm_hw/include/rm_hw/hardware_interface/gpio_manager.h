//
// Created by yezi on 2021/9/9.
//

#pragma once

#include <XmlRpcValue.h>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <rm_common/hardware_interface/gpio_interface.h>

namespace rm_hw
{
class GpioManager
{
public:
  explicit GpioManager(std::mutex& mutex) : gpio_mutex_(mutex)
  {
    gpio_thread_ = std::thread([&]() {
      while (ros::ok())
        gpioUpdate();
    });
  };
  ~GpioManager()
  {
    if (gpio_thread_.joinable())
      gpio_thread_.join();
  };

  void setGpioDirection(rm_control::GpioData gpioData);
  void gpioUpdate();

  std::vector<rm_control::GpioData> gpio_state_values;
  std::vector<rm_control::GpioData> gpio_command_values;

private:
  void readGpio();
  void writeGpio();

  std::thread gpio_thread_;
  std::mutex& gpio_mutex_;
};
}  // namespace rm_hw
