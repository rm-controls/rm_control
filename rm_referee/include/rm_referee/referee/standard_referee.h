//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/referee/hero_referee.h"

namespace rm_referee
{
class StandardReferee : public HeroReferee
{
public:
  explicit StandardReferee(ros::NodeHandle& nh, Base& base);

  void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data) override;

protected:
};
}  // namespace rm_referee
