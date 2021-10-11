//
// Created by yezi on 2021/9/9.
//

#include <rm_hw/hardware_interface/gpio_manager.h>

GpioMangager::GpioMangager()
{
  map_outputio_.clear();
  map_inputio_.clear();
}

GpioMangager::~GpioMangager()
{
  if (!map_outputio_.empty())
  {
    for (auto i : map_outputio_)
    {
      close(i.second);
    }
  }
  if (!map_inputio_.empty())
  {
    for (auto i : map_inputio_)
    {
      close(i.second);
    }
  }
  map_inputio_.clear();
  map_outputio_.clear();
}

void GpioMangager::writeOutput(int pin, bool IS_HIGH)
{
  lseek(map_outputio_[pin], 0, SEEK_SET);
  if (IS_HIGH)
  {
    int ref = write(map_outputio_[pin], "1", 1);
    if (ref == -1)
      ROS_ERROR("[GPIO]Failed to write to GPIO%d.", pin);
  }
  else
  {
    int ref = write(map_outputio_[pin], "0", 1);
    if (ref == -1)
      ROS_ERROR("[GPIO]Failed to write to GPIO%d.", pin);
  }
}

void GpioMangager::writeOutput(std::vector<GpioData>& gpio_write)
{
  for (const auto& iter : gpio_write)
  {
    lseek(map_outputio_[map_name2pin_[iter.name]], 0, SEEK_SET);
    if (iter.value)
    {
      int ref = write(map_outputio_[map_name2pin_[iter.name]], "1", 1);
      if (ref == -1)
        ROS_ERROR("[GPIO]Failed to write to GPIO%d.", map_name2pin_[iter.name]);
    }
    else
    {
      int ref = write(map_outputio_[map_name2pin_[iter.name]], "0", 1);
      if (ref == -1)
        ROS_ERROR("[GPIO]Failed to write to GPIO%d.", map_name2pin_[iter.name]);
    }
  }
}

void GpioMangager::readInput(std::vector<GpioDataStamp>& gpio_read_stamp)
{
  int j = 0;
  for (auto iter : map_inputio_)
  {
    fds[j].fd = iter.second;
    fds[j].events = POLLPRI;
    j++;
  }
  char state;
  int ret = poll(fds, map_inputio_.size(), 0);
  if (ret == -1)
  {
    ROS_ERROR("poll failed!\n");
  }
  for (unsigned int i = 0; i < map_inputio_.size(); i++)
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
    gpio_read_stamp[i].data.value = value;
    gpio_read_stamp[i].stamp = ros::Time::now();
  }
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
  map_inputio_.insert(std::make_pair(pin, fd));
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
  map_outputio_.insert(std::make_pair(pin, fd));
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
