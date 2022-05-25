//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/common/referee_base.h"

namespace rm_referee
{
RefereeBase::RefereeBase(ros::NodeHandle& nh) : data_(nh), nh_(nh)
{
}

void RefereeBase::run()
{
  try
  {
    if (data_.serial_.available())
    {
      data_.referee_.rx_len_ = (int)data_.serial_.available();
      data_.serial_.read(data_.referee_.rx_buffer_, data_.referee_.rx_len_);
    }
  }
  catch (serial::IOException& e)
  {
  }
  data_.referee_.read();
  try
  {
    data_.serial_.write(data_.referee_.tx_buffer_, data_.referee_.tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
  }
  data_.referee_.clearBuffer();
}

}  // namespace rm_referee
