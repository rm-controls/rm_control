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

  bool add_ui_flag;
  TimeChangeUi* time_change_ui_{};
  FlashUi* flash_ui_{};
  TriggerChangeUi* trigger_change_ui_{};
  FixedUi* fixed_ui_{};

private:
  double capacitor_threshold, extra_power, burst_power;
};
}  // namespace rm_referee
