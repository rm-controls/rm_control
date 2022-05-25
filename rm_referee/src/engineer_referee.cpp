//
// Created by ljq on 2022/5/17.
//
#include "rm_referee/engineer_referee.h"

namespace rm_referee
{
EngineerReferee::EngineerReferee(ros::NodeHandle& nh) : RobotReferee(nh)
{
}

void EngineerReferee::run()
{
  RobotReferee::run();
}

void EngineerReferee::drawUi(const ros::Time& time)
{
  RobotReferee::drawUi(time);
  time_change_ui_->update("effort", time);
  time_change_ui_->update("temperature", time);
  trigger_change_ui_->update("card", 0, data_.card_cmd_data_.mode);
  flash_ui_->update("calibration", time, data_.calibration_status_data_.calibration_state);
  if (!data_.joint_state_.name.empty())
    flash_ui_->update("card_warning", time, data_.joint_state_.effort[0] < 1.5);
  //    trigger_change_ui_->update("jog", jog_joint_name);
  drawProcess(time);
}

void EngineerReferee::drawProcess(const ros::Time& time)
{
  if (data_.engineer_cmd_data_.symbol != symbol)
  {
    if (data_.engineer_cmd_data_.total_steps != 0)
      time_change_ui_->update("progress", ros::Time::now(),
                              data_.engineer_cmd_data_.finished_step / data_.engineer_cmd_data_.total_steps);
    else
      time_change_ui_->update("progress", ros::Time::now(), 0.);
  }
  symbol = data_.engineer_cmd_data_.symbol;
  trigger_change_ui_->update("step", data_.engineer_cmd_data_.step_queue_name);
}

}  // namespace rm_referee
