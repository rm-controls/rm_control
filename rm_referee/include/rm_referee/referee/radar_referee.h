//
// Created by yuchen on 2022/6/10.
//

#pragma once

#include "rm_referee/referee/referee_base.h"
#include <std_msgs/Int8MultiArray.h>

namespace rm_referee
{
class RadarReferee : public RefereeBase
{
public:
  explicit RadarReferee(ros::NodeHandle& nh, Base& base);
  void run() override;

private:
  std::vector<int> red_receiver_ = {
    1,  // hero
    2,  // engineer
    3,  // standard3
    4,  // standard4
    5,  // standard5
  };
  std::vector<int> blue_receiver_ = {
    101,  // hero
    102,  // engineer
    103,  // standard3
    104,  // standard4
    105,  // standard5
  };
};
}  // namespace rm_referee
