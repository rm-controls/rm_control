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

struct GpioData
{
  std::string name;
  bool value;
};

class GpioMangager
{
public:
  explicit GpioMangager();
  ~GpioMangager();
  void writeOutput();
  void readInput();
  void addInIo(int pin);
  void addOutIo(int pin);
  void ioDirectionSet(const std::string& pin, bool IS_OUT);
  std::map<std::string, int> map_name2pin_;
  std::map<int, int> map_outputio_;
  std::map<int, int> map_inputio_;
  std::vector<GpioData> gpio_read_values;
  std::vector<GpioData> gpio_write_values;

private:
  std::string pin_;
  struct pollfd fds[20]{};
};
