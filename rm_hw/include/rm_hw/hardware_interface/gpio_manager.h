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

enum GpioType
{
  GPIO_INPUT,
  GPIO_OUTPUT
};

struct GpioData
{
  std::string name;
  GpioType type;
  int pin;
  bool* value;
};

class GpioMangager
{
public:
  explicit GpioMangager();
  ~GpioMangager();

  void setGpioDirection(GpioData gpioData);
  void readGpio();
  void writeGpio();

  std::vector<GpioData> gpio_state_values;
  std::vector<GpioData> gpio_command_values;
};
