//
// Created by ljq on 2022/5/17.
//
#include "rm_referee/engineer_referee.h"

namespace rm_referee
{
EngineerReferee::EngineerReferee(ros::NodeHandle& nh, Base& base) : RobotReferee(nh, base)
{
  EngineerReferee::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &EngineerReferee::jointStateCallback, this);
  EngineerReferee::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &EngineerReferee::manualDataCallBack, this);
  interactive_data_sender_ = new Graph(base_);
}

void EngineerReferee::run()
{
  RobotReferee::run();
}

void EngineerReferee::interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data_,
                                              const ros::Time& last_get_)
{
  RefereeBase::interactiveDataCallBack(interactive_data_, last_get_);
}
void EngineerReferee::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  RefereeBase::jointStateCallback(data);
  time_change_ui_->update("effort", ros::Time::now());
  if (!base_.joint_state_.name.empty())
    flash_ui_->update("card_warning", ros::Time::now(), base_.joint_state_.effort[0] < 1.5);
  //    trigger_change_ui_->update("jog", jog_joint_name);
}

void EngineerReferee::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
  RefereeBase::actuatorStateCallback(data);
  time_change_ui_->update("temperature", ros::Time::now());
}

void EngineerReferee::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
  RefereeBase::cardCmdDataCallback(data);
  trigger_change_ui_->update("card", 0, base_.card_cmd_data_.mode);
}

void EngineerReferee::engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data)
{
  if (base_.engineer_cmd_data_.step_queue_name != data->step_queue_name)
  {
    if (base_.engineer_cmd_data_.total_steps != 0)
      time_change_ui_->update("progress", ros::Time::now(),
                              base_.engineer_cmd_data_.finished_step / base_.engineer_cmd_data_.total_steps);
    else
      time_change_ui_->update("progress", ros::Time::now(), 0.);
  }
  trigger_change_ui_->update("step", base_.engineer_cmd_data_.step_queue_name);
  RefereeBase::engineerCmdDataCallback(data);
}

void EngineerReferee::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  RefereeBase::manualDataCallBack(data);
  flash_ui_->update("calibration", ros::Time::now(), base_.manual_to_referee_data_.engineer_calibration_state);
}

}  // namespace rm_referee
