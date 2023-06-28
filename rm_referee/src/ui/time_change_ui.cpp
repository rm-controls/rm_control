//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/time_change_ui.h"

namespace rm_referee
{
void TimeChangeUi::update()
{
  updateConfig();
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  UiBase::display(ros::Time::now());
}

void TimeChangeGroupUi::update()
{
  updateConfig();
  for (auto graph : graph_vector_)
    graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
  for (auto character : character_vector_)
    character.second->setOperation(rm_referee::GraphOperation::UPDATE);
  display(ros::Time::now());
}

void TimeChangeUi::updateForQueue()
{
  updateConfig();
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);

  if (graph_queue_ && !graph_->isRepeated() && ros::Time::now() - last_send_ > delay_)
  {
    graph_queue_->push_back(*graph_);
    last_send_ = ros::Time::now();
  }
}

void TimeChangeGroupUi::updateForQueue()
{
  updateConfig();
  if (ros::Time::now() - last_send_ > delay_)
    for (auto graph : graph_vector_)
    {
      graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
      if (graph_queue_ && !graph.second->isRepeated())
      {
        graph_queue_->push_back(*graph.second);
        last_send_ = ros::Time::now();
      }
    }
}

void CapacitorTimeChangeUi::add()
{
  if (remain_charge_ != 0.)
    graph_->setOperation(rm_referee::GraphOperation::ADD);
  UiBase::display(false);
}

void CapacitorTimeChangeUi::updateConfig()
{
  if (remain_charge_ > 0.)
  {
    graph_->setStartX(610);
    graph_->setStartY(100);
    graph_->setEndX(610 + 600 * remain_charge_);
    graph_->setEndY(100);
    if (remain_charge_ > 0.7)
      graph_->setColor(rm_referee::GraphColor::GREEN);
    else if (remain_charge_ > 0.3)
      graph_->setColor(rm_referee::GraphColor::ORANGE);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  else
    return;
}

void CapacitorTimeChangeUi::updateRemainCharge(const double remain_charge, const ros::Time& time)
{
  remain_charge_ = remain_charge;
  updateForQueue();
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
      TimeChangeUi::update();
    }
  }
}

void ProgressTimeChangeUi::updateConfig()
{
  char data_str[30] = { ' ' };
  if (total_steps_ != 0)
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  else
    sprintf(data_str, " %.1f%%", finished_data_ / total_steps_ * 100.);
  graph_->setContent(data_str);
}

void ProgressTimeChangeUi::updateEngineerUiData(const rm_msgs::EngineerUi::ConstPtr data,
                                                const ros::Time& last_get_data_time)
{
  total_steps_ = data->total_steps;
  finished_data_ = data->finished_step;
  TimeChangeUi::update();
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
}

void DartStatusTimeChangeUi::updateDartClientCmd(const rm_msgs::DartClientCmd::ConstPtr data,
                                                 const ros::Time& last_get_data_time)
{
  dart_launch_opening_status_ = data->dart_launch_opening_status;
  TimeChangeUi::update();
}

void RotationTimeChangeUi::updateConfig()
{
  if (!tf_buffer_.canTransform(gimbal_reference_frame_, chassis_reference_frame_, ros::Time(0)))
    return;
  try
  {
    int angle;
    double roll, pitch, yaw;
    quatToRPY(
        tf_buffer_.lookupTransform(chassis_reference_frame_, gimbal_reference_frame_, ros::Time(0)).transform.rotation,
        roll, pitch, yaw);
    angle = static_cast<int>(yaw * 180 / M_PI);

    graph_->setStartAngle((angle - arc_scale_ / 2) % 360 < 0 ? (angle - arc_scale_ / 2) % 360 + 360 :
                                                               (angle - arc_scale_ / 2) % 360);
    graph_->setEndAngle((angle + arc_scale_ / 2) % 360 < 0 ? (angle + arc_scale_ / 2) % 360 + 360 :
                                                             (angle + arc_scale_ / 2) % 360);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void LaneLineTimeChangeGroupUi::updateConfig()
{
  double spacing_x_a = robot_radius_ * screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) /
                       (cos(end_point_a_angle_ - pitch_angle_) * robot_height_ / sin(end_point_a_angle_)),
         spacing_x_b = robot_radius_ * screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) /
                       (cos(end_point_b_angle_ - pitch_angle_) * robot_height_ / sin(end_point_b_angle_)),
         spacing_y_a = screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) * tan(end_point_a_angle_ - pitch_angle_),
         spacing_y_b = screen_y_ / 2 * tan(M_PI / 2 - camera_range_ / 2) * tan(end_point_b_angle_ - pitch_angle_);
  if (spacing_x_a < 0)
    return;

  if (spacing_x_b < 0)
    return;

  for (auto it : graph_vector_)
  {
    if (it.first == "lane_line_left")
    {
      it.second->setStartX(screen_x_ / 2 - spacing_x_a);
      it.second->setStartY(screen_y_ / 2 - spacing_y_a);
      it.second->setEndX(screen_x_ / 2 - spacing_x_b * surface_coefficient_);
      it.second->setEndY(screen_y_ / 2 - spacing_y_b);
    }
    else if (it.first == "lane_line_right")
    {
      it.second->setStartX(screen_x_ / 2 + spacing_x_a);
      it.second->setStartY(screen_y_ / 2 - spacing_y_a);
      it.second->setEndX(screen_x_ / 2 + spacing_x_b * surface_coefficient_);
      it.second->setEndY(screen_y_ / 2 - spacing_y_b);
    }
  }
}

void LaneLineTimeChangeGroupUi::updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time)
{
  for (unsigned int i = 0; i < data->name.size(); i++)
    if (data->name[i] == reference_joint_)
      pitch_angle_ = data->position[i];

  end_point_a_angle_ = camera_range_ / 2 + pitch_angle_;
  end_point_b_angle_ = 0.6 * (0.25 + pitch_angle_);
  updateForQueue();
}

void BalancePitchTimeChangeGroupUi::updateConfig()
{
  for (auto it : graph_vector_)
  {
    if (it.first == "triangle_left_side")
    {
      it.second->setStartX(centre_point_[0]);
      it.second->setStartY(centre_point_[1]);
      it.second->setEndX(triangle_left_point_[0]);
      it.second->setEndY(triangle_left_point_[1]);
    }
    else if (it.first == "triangle_right_side")
    {
      it.second->setStartX(centre_point_[0]);
      it.second->setStartY(centre_point_[1]);
      it.second->setEndX(triangle_right_point_[0]);
      it.second->setEndY(triangle_right_point_[1]);
    }
  }
}

void BalancePitchTimeChangeGroupUi::calculatePointPosition(const rm_msgs::BalanceStateConstPtr& data,
                                                           const ros::Time& time)
{
  triangle_left_point_[0] = centre_point_[0] - length_ * sin(bottom_angle_ / 2 + data->theta);
  triangle_left_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2 + data->theta);
  triangle_right_point_[0] = centre_point_[0] + length_ * sin(bottom_angle_ / 2 - data->theta);
  triangle_right_point_[1] = centre_point_[1] + length_ * cos(bottom_angle_ / 2 - data->theta);
  updateForQueue();
}

void PitchAngleTimeChangeUi::updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time)
{
  for (unsigned int i = 0; i < data->name.size(); i++)
    if (data->name[i] == "pitch_joint")
      pitch_angle_ = data->position[i];
  update();
}

void PitchAngleTimeChangeUi::update()
{
  updateConfig();
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  display(ros::Time::now());
}

void PitchAngleTimeChangeUi::updateConfig()
{
  std::string pitch = std::to_string(pitch_angle_);
  graph_->setContent(pitch);
  graph_->setColor(rm_referee::GraphColor::YELLOW);
}

void JointPositionTimeChangeUi::updateConfig()
{
  double proportion = (current_val_ - min_val_) / (max_val_ - min_val_);
  graph_->setStartX(graph_->getConfig().start_x);
  graph_->setStartY(graph_->getConfig().start_y);
  if (direction_ == "horizontal")
  {
    graph_->setEndY(graph_->getConfig().start_y);
    graph_->setEndX(graph_->getConfig().start_x + length_ * proportion);
  }
  if (direction_ == "vertical")
  {
    graph_->setEndY(graph_->getConfig().start_y + length_ * proportion);
    graph_->setEndX(graph_->getConfig().end_x);
  }
  if (direction_ == "slant")
  {
    graph_->setEndY(graph_->getConfig().start_y + length_ * proportion);
    graph_->setEndX(graph_->getConfig().start_x + length_ * proportion);
  }
  if (abs(proportion) > 0.96)
    graph_->setColor(rm_referee::GraphColor::BLACK);
  else if (abs(proportion) > 0.8)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else if (abs(proportion) > 0.6)
    graph_->setColor(rm_referee::GraphColor::PURPLE);
  else if (abs(proportion) > 0.3)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else
    graph_->setColor(rm_referee::GraphColor::GREEN);
}

void JointPositionTimeChangeUi::updateJointStateData(const sensor_msgs::JointState::ConstPtr data, const ros::Time& time)
{
  for (unsigned int i = 0; i < data->name.size(); i++)
    if (data->name[i] == name_)
      current_val_ = data->position[i];
  updateForQueue();
}

void SpaceTfTimeChangeGroupUi::updateConfig()
{
  if (!tf_buffer_.canTransform("base_link", "tools_link", ros::Time(0)))
    return;
  try
  {
    geometry_msgs::TransformStamped tool2base;
    tool2base = tf_buffer_.lookupTransform("base_link", "tools_link", ros::Time(0));
    double roll, pitch, yaw;
    quatToRPY(tool2base.transform.rotation, roll, pitch, yaw);
    tf_info_[0] = tool2base.transform.translation.x;
    tf_info_[1] = tool2base.transform.translation.y;
    tf_info_[2] = tool2base.transform.translation.z;
    tf_info_[3] = roll;
    tf_info_[4] = pitch;
    tf_info_[5] = yaw;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  calculateTransformedEndpoint(start_point_, store_end_points_, tf_info_[3], tf_info_[4], tf_info_[5]);
  std::vector<double> proportions{ 0., 0., 0., 0., 0., 0. };
  for (int i = 0; i < (int)range_gather_.size(); ++i)
  {
    proportions[i] = (tf_info_[i] - range_gather_[i][0]) / (range_gather_[i][1] - range_gather_[i][0]);
  }
  int times = 0;
  for (auto it = graph_vector_.begin(); it != graph_vector_.end(); ++it)
  {
    it->second->setStartX(start_point_.x);
    it->second->setStartY(start_point_.y);
    it->second->setEndX(vision_points_[times].x);
    it->second->setEndY(vision_points_[times].y);
    if (it->first == "x")
      it->second->setColor(rm_referee::GraphColor::PINK);
    else if (it->first == "y")
      it->second->setColor(rm_referee::GraphColor::GREEN);
    else if (it->first == "z")
      it->second->setColor(rm_referee::GraphColor::CYAN);
    times++;
  }
}

void SpaceTfTimeChangeGroupUi::calculateTransformedEndpoint(const Vector2D& start_point,
                                                            std::vector<Vector2D>& end_points, double roll,
                                                            double pitch, double yaw)
{
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix.setIdentity();
  rotationMatrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * rotationMatrix;
  rotationMatrix = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * rotationMatrix;
  rotationMatrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * rotationMatrix;

  Eigen::Vector3d endpointVector(xyz_length_[0], 0., 0.);
  Eigen::Vector3d transformedEndpointVector = rotationMatrix * endpointVector;

  for (int i = 0; i < (int)store_end_points_.size(); ++i)
  {
    store_end_points_[i].x = end_points_[i].x - transformedEndpointVector.y();
    store_end_points_[i].y = end_points_[i].y + transformedEndpointVector.z();
  }
  double scaleFactor = 1.;
  for (int i = 0; i < (int)end_points_.size(); ++i)
  {
    vision_points_[i].x = store_end_points_[i].x * scaleFactor;
    vision_points_[i].y = store_end_points_[i].y * scaleFactor;
  }
}
}  // namespace rm_referee
