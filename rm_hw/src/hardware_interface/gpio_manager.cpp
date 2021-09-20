//
// Created by yezi on 2021/9/9.
//

#include <rm_hw/hardware_interface/gpio_manager.h>

GpioMangager::GpioMangager()
{
  mapOutputIo_.clear();
  mapInputIo_.clear();
}

GpioMangager::~GpioMangager()
{
  if (!mapOutputIo_.empty())
  {
    for (auto i : mapOutputIo_)
    {
      close(i.second);
    }
  }
  if (!mapInputIo_.empty())
  {
    for (auto i : mapInputIo_)
    {
      close(i.second);
    }
  }
  mapInputIo_.clear();
  mapOutputIo_.clear();
}

bool GpioMangager::init(const ros::NodeHandle& module_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (!module_nh.getParam("gpios", xml_rpc_value))
    ROS_ERROR("Initialize failed.");
  for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
  {
    if (it->second.hasMember("pin"))
    {
      if (it->second.hasMember("direction"))
      {
        std::string direction = xml_rpc_value[it->first]["direction"];
        std::string::size_type idx;
        idx = direction.find("out");
        if (idx == std::string::npos)
        {
          idx = direction.find("in");
          if (idx == std::string::npos)
            ROS_ERROR("Module %s hasn't set direction.", it->first.data());
          else
            addInIo(xml_rpc_value[it->first]["pin"]);
        }
        else
          addOutIo(xml_rpc_value[it->first]["pin"]);
      }
      else
      {
        ROS_ERROR("Module %s hasn't set direction.", it->first.data());
      }
    }
    else
    {
      ROS_ERROR("Module %s hasn't set pin ID", it->first.data());
    }
  }
  return true;
}

void GpioMangager::addInIo(int pin)
{
  int fd;
  pin_ = std::to_string(pin);

  ioDirectionSet(pin_, false);

  std::string file = "/sys/class/gpio/gpio" + pin_ + "/value";
  fd = open(file.data(), O_WRONLY);
  if (fd == -1)
  {
    ROS_ERROR("[gpio]Unable to open /sys/class/gpio/gpio%i/value", pin);
  }
  mapInputIo_.insert(std::make_pair(pin, fd));
}

void GpioMangager::addOutIo(int pin)
{
  int fd;
  pin_ = std::to_string(pin);

  ioDirectionSet(pin_, true);

  std::string file = "/sys/class/gpio/gpio" + pin_ + "/value";
  fd = open(file.data(), O_WRONLY);
  if (fd == -1)
  {
    ROS_ERROR("[gpio]Unable to open /sys/class/gpio/gpio%i/value", pin);
  }
  mapOutputIo_.insert(std::make_pair(pin, fd));
}

void GpioMangager::writeOutput(int pin, bool IS_HIGH)
{
  lseek(mapOutputIo_[pin], 0, SEEK_SET);
  if (IS_HIGH)
  {
    int ref = write(mapOutputIo_[pin], "1", 1);
    if (ref == -1)
      ROS_ERROR("[GPIO]Failed to write to GPIO%d.", pin);
  }
  else
  {
    int ref = write(mapOutputIo_[pin], "0", 1);
    if (ref == -1)
      ROS_ERROR("[GPIO]Failed to write to GPIO%d.", pin);
  }
}

void GpioMangager::readInput(std::vector<GpioDataStamp>& gpio_data_stamp_vector)
{
  int j = 0;
  for (auto iter : mapInputIo_)
  {
    fds[j].fd = iter.second;
    fds[j].events = POLLPRI;
    j++;
  }
  char state;
  int ret = poll(fds, mapInputIo_.size(), 0);
  if (ret == -1)
  {
    ROS_ERROR("poll failed!\n");
  }
  std::map<int, int>::iterator it = mapInputIo_.begin();
  std::vector<GpioDataStamp>::iterator iter = gpio_data_stamp_vector.begin();
  for (int i = 0; i < mapInputIo_.size(); i++)
  {
    if (fds[i].revents & POLLPRI)
    {
      int gpio_fd = fds[i].fd;
      lseek(gpio_fd, 0, SEEK_SET);
      ret = read(gpio_fd, &state, 1);
      if (ret == -1)
      {
        ROS_ERROR("[gpio]:run :read failed");
        break;
      }
    }
    bool value = (state == 0x31);
    GpioDataStamp gpio_data_stamp;
    gpio_data_stamp.data.pin = it->first;
    gpio_data_stamp.data.value = value;
    gpio_data_stamp.stamp = ros::Time::now();
    gpio_data_stamp_vector.push_back(gpio_data_stamp);
    ++it;
    ++iter;
  }
}

void GpioMangager::ioDirectionSet(const std::string& pin, bool IS_OUT)
{
  std::string file = "/sys/class/gpio/gpio" + pin + "/direction";
  int fd;
  fd = open(file.data(), O_WRONLY);
  if (fd == -1)
  {
    ROS_ERROR("[gpio]Unable to open %s", file.data());
  }
  if (IS_OUT)
  {
    if (write(fd, "out", 3) != 3)
    {
      ROS_ERROR("[gpio]Failed to set direction of gpio%s", pin.data());
    }
  }
  else
  {
    if (write(fd, "in", 2) != 2)
    {
      ROS_ERROR("[gpio]Failed to set direction of gpio%s", pin.data());
    }
  }
  close(fd);
}