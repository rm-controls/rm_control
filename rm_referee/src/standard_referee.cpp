//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/standard_referee.h"

namespace rm_referee
{
void StandardReferee::run()
{
  RobotReferee::run();
}

void StandardReferee::drawUi(const ros::Time& time)
{
  HeroReferee::drawUi(time);
  flash_ui_->update("cover", time, !data_.cover_cmd_data_.mode);
}
}  // namespace rm_referee
