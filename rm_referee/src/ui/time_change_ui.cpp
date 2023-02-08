//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/time_change_ui.h"

namespace rm_referee
{
void TimeChangeUi::display(const ros::Time& time)
{
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display(time);
  graph_->sendUi(ros::Time::now());
}

void CapacitorTimeChangeUi::add()
{
  if (cap_power_ != 0.)
  {
    graph_->setOperation(rm_referee::GraphOperation::ADD);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }
}

void CapacitorTimeChangeUi::display(const ros::Time& time)
{
  updateConfig();
  TimeChangeUi::display(time);
}

void CapacitorTimeChangeUi::updateConfig()
{
  if (cap_power_ > 0.)
  {
    graph_->setStartX(610);
    graph_->setStartY(100);
    graph_->setEndX(610 + 600 * cap_power_);
    graph_->setEndY(100);
    if (cap_power_ < 0.3)
      graph_->setColor(rm_referee::GraphColor::ORANGE);
    else if (cap_power_ > 0.7)
      graph_->setColor(rm_referee::GraphColor::GREEN);
    else
      graph_->setColor(rm_referee::GraphColor::YELLOW);
  }
  else
    return;
}

void CapacitorTimeChangeUi::updateCapacityData(const rm_msgs::CapacityData data, const ros::Time& time)
{
  cap_power_ = data.cap_power;
  display(time);
}

void EffortTimeChangeUi::display(const ros::Time& time)
{
  updateConfig();
  TimeChangeUi::display(time);
}

void EffortTimeChangeUi::updateConfig()
{
  char data_str[30] = { ' ' };
  sprintf(data_str, "%s:%.2f N.m", joint_name_.c_str(), joint_effort_);
  graph_->setContent(data_str);
  if (joint_effort_ > 20.)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (joint_effort_ < 10.)
    graph_->setColor(rm_referee::GraphColor::GREEN);
  else
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void EffortTimeChangeUi::updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time)
{
  int max_index = 0;
  if (!data->name.empty())
  {
    for (int i = 0; i < static_cast<int>(data->effort.size()); ++i)
      if ((data->name[i] == "joint1" || data->name[i] == "joint2" || data->name[i] == "joint3" ||
           data->name[i] == "joint4" || data->name[i] == "joint5") &&
          data->effort[i] > data->effort[max_index])
        max_index = i;
    if (max_index != 0)
    {
      joint_effort_ = data->effort[max_index];
      joint_name_ = data->name[max_index];
      display(time);
    }
  }
}

void ProgressTimeChangeUi::display(const ros::Time& time)
{
  updateConfig();
  TimeChangeUi::display(time);
}

void ProgressTimeChangeUi::updateConfig()
{
  char data_str[30] = { ' ' };
  if (total_steps_ != 0)
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  else
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  graph_->setContent(data_str);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void ProgressTimeChangeUi::updateStepQueueStateData(const rm_msgs::StepQueueState::ConstPtr data,
                                                    const ros::Time& last_get_data_time)
{
  total_steps_ = data->total_steps;
  finished_data_ = data->finished_step;
  display(last_get_data_time);
}

void DartStatusTimeChangeUi::display(const ros::Time& time)
{
  updateConfig();
  TimeChangeUi::display(time);
}

void DartStatusTimeChangeUi::updateConfig()
{
  char data_str[30] = { ' ' };
  if (dart_launch_opening_status_ == 1)
  {
    sprintf(data_str, "Dart Status: Close");
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  }
  else if (dart_launch_opening_status_ == 2)
  {
    sprintf(data_str, "Dart Status: Changing");
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  }
  else if (dart_launch_opening_status_ == 0)
  {
    sprintf(data_str, "Dart Open!");
    graph_->setColor(rm_referee::GraphColor::GREEN);
  }
  graph_->setContent(data_str);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
}

void DartStatusTimeChangeUi::updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data,
                                                 const ros::Time& last_get_data_time)
{
  dart_launch_opening_status_ = data->dart_launch_opening_status;
  display(last_get_data_time);
}

void LaneLineTimeChangeUi::add()
{
  graph_left_->setOperation(rm_referee::GraphOperation::ADD);
  graph_left_->display(true);
  graph_left_->sendUi(ros::Time::now());
  graph_right_->setOperation(rm_referee::GraphOperation::ADD);
  graph_right_->display(true);
  graph_right_->sendUi(ros::Time::now());
}
void LaneLineTimeChangeUi::display(const ros::Time& time)
{
  updateConfig();

  graph_left_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_left_->display(ros::Time::now());
  graph_left_->sendUi(ros::Time::now());
  graph_right_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_right_->display(ros::Time::now());
  graph_right_->sendUi(ros::Time::now());
}

void LaneLineTimeChangeUi::updateConfig()
{
  double spacing_x_a = robot_radius_ * screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) /
                       (cos(end_point_a_angle_ - pitch_angle_) * robot_height_ / sin(end_point_a_angle_)),
         spacing_x_b = robot_radius_ * screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) /
                       (cos(end_point_b_angle_ - pitch_angle_) * robot_height_ / sin(end_point_b_angle_)),
         spacing_y_a = screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) * tan(end_point_a_angle_ - pitch_angle_),
         spacing_y_b = screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) * tan(end_point_b_angle_ - pitch_angle_);

  graph_left_->setStartX(screen_x_ / 2 - spacing_x_a);
  graph_left_->setStartY(screen_y_ / 2 - spacing_y_a);
  graph_left_->setEndX(screen_x_ / 2 - spacing_x_b * surface_coefficient_);
  graph_left_->setEndY(screen_y_ / 2 - spacing_y_b);

  graph_right_->setStartX(screen_x_ / 2 + spacing_x_a);
  graph_right_->setStartY(screen_y_ / 2 - spacing_y_a);
  graph_right_->setEndX(screen_x_ / 2 + spacing_x_b * surface_coefficient_);
  graph_right_->setEndY(screen_y_ / 2 - spacing_y_b);
}

void LaneLineTimeChangeUi::updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time)
{
  for (unsigned int i = 0; i < data->name.size(); i++)
    if (data->name[i] == reference_joint_)
      pitch_angle_ = data->position[i];

  end_point_a_angle_ = camera_range_ / 2 + pitch_angle_;
  end_point_b_angle_ = 0.6 * (0.25 + pitch_angle_);
  display(time);
}
}  // namespace rm_referee
