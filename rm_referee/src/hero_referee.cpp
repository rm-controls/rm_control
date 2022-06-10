//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/hero_referee.h"

namespace rm_referee
{
void HeroReferee::run()
{
  RobotReferee::run();
}

void HeroReferee::drawUi(const ros::Time& time)
{
  RobotReferee::drawUi(time);
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    if (data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::BLUE_HERO &&
        data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::RED_HERO)
      trigger_change_ui_->update("target", data_.detection_status_data_.target,
                                 data_.chassis_cmd_data_.power_limit_state, data_.detection_status_data_.armor_target,
                                 data_.detection_status_data_.color == rm_msgs::StatusChangeRequest::RED);
    else
      trigger_change_ui_->update("target", gimbal_eject ? 1 : 0, data_.chassis_cmd_data_.power_limit_state,
                                 data_.detection_status_data_.armor_target,
                                 data_.detection_status_data_.color == rm_msgs::StatusChangeRequest::RED);
    trigger_change_ui_->update("exposure", data_.detection_status_data_.exposure, false);
    fixed_ui_->update();
    //flash_ui_->update("aux", time, false);
  }
}
}  // namespace rm_referee