//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/hero_referee.h"

namespace rm_referee
{
class StandardReferee : public HeroReferee
{
public:
  explicit StandardReferee(ros::NodeHandle& nh) : HeroReferee(nh){};
  void run() override;

protected:
  void drawUi(const ros::Time& time) override;
};
}  // namespace rm_referee