//
// Created by yezi on 2021/9/9.
//

#ifndef SRC_CHASSIS_BASE_INCLUDE_GPIO_MANAGER_H_
#define SRC_CHASSIS_BASE_INCLUDE_GPIO_MANAGER_H_

#include <XmlRpcValue.h>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <ros/ros.h>
#include <string>

struct GpioData
{
  std::string name;
  int value;
};

struct GpioDataStamp
{
  GpioData data{};
  ros::Time stamp;
};

class GpioMangager
{
public:
  explicit GpioMangager();
  ~GpioMangager();
  bool init(const ros::NodeHandle& module_nh);
  void writeOutput(int pin, bool IS_HIGH);
  void writeOutput(std::vector<GpioDataStamp>& gpio_write_stamp);
  void readInput(std::vector<GpioDataStamp>& gpio_read_stamp);
  void addInIo(int pin);
  void addOutIo(int pin);
  void ioDirectionSet(const std::string& pin, bool IS_OUT);
  std::map<std::string, int> mapName2Pin_;
  std::map<int, int> mapOutputIo_;
  std::map<int, int> mapInputIo_;

private:
  std::string pin_;
  struct pollfd fds[20]{};
};

#endif  // SRC_CHASSIS_BASE_INCLUDE_GPIO_MANAGER_H_
