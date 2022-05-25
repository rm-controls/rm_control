//
// Created by ljq on 2022/5/17.
//

#pragma once

#include "rm_referee/common/referee_base.h"

namespace rm_referee
{
class RobotReferee : public RefereeBase
{
public:
  explicit RobotReferee(ros::NodeHandle& nh);

protected:
  void drawUi(const ros::Time& time) override;

  TimeChangeUi* time_change_ui_{};
  FlashUi* flash_ui_{};
  TriggerChangeUi* trigger_change_ui_{};
  FixedUi* fixed_ui_{};
};
}  // namespace rm_referee
