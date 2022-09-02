//
// Created by yezi on 2021/9/9.
//

#include <rm_hw/hardware_interface/gpio_manager.h>

namespace rm_hw
{
GpioManager::GpioManager()
{
}

GpioManager::~GpioManager()
{
}
void GpioManager::setGpioDirection(rm_control::GpioData gpioData)
{
  std::string file = "/sys/class/gpio/gpio" + std::to_string(gpioData.pin) + "/direction";
  int fd;
  fd = open(file.data(), O_WRONLY);
  if (fd == -1)
  {
    ROS_ERROR("[gpio]Unable to open %s", file.data());
  }
  else
  {
    if (gpioData.type == rm_control::OUTPUT)
    {
      if (write(fd, "out", 3) != 3)
      {
        ROS_ERROR("[gpio]Failed to set direction of gpio%d", gpioData.pin);
      }
    }
    else
    {
      if (write(fd, "in", 2) != 2)
      {
        ROS_ERROR("[gpio]Failed to set direction of gpio%d", gpioData.pin);
      }
    }
  }
  close(fd);
}

void GpioManager::readGpio()
{
  for (auto iter = gpio_state_values.begin(); iter != gpio_state_values.end(); iter++)
  {
    if (iter->type == rm_control::INPUT)
    {
      std::string file = "/sys/class/gpio/gpio" + std::to_string(iter->pin) + "/value";
      FILE* fp = fopen(file.c_str(), "r");
      if (fp == NULL)
      {
        ROS_ERROR("[gpio]Unable to read /sys/class/gpio/gpio%d/value", iter->pin);
      }
      else
      {
        char state = fgetc(fp);
        bool value = (state == 0x31);
        *iter->value = value;
        fclose(fp);
      }
    }
  }
}

void GpioManager::writeGpio()
{
  char buffer[1] = { '1' };
  for (auto iter : gpio_command_values)
  {
    std::string file = "/sys/class/gpio/gpio" + std::to_string(iter.pin) + "/value";
    int fd = open(file.c_str(), O_WRONLY);
    if (fd == -1)
    {
      ROS_ERROR("[gpio]Unable to write /sys/class/gpio/gpio%i/value", iter.pin);
    }
    else
    {
      lseek(fd, 0, SEEK_SET);
      if (*iter.value)
      {
        buffer[0] = '1';
        int ref = write(fd, buffer, 1);
        if (ref == -1)
          ROS_ERROR("[GPIO]Failed to write to gpio%d.", iter.pin);
      }
      else
      {
        buffer[0] = '0';
        int ref = write(fd, buffer, 1);
        if (ref == -1)
          ROS_ERROR("[GPIO]Failed to write to gpio%d.", iter.pin);
      }
    }
    close(fd);
  }
}
}  // namespace rm_hw
