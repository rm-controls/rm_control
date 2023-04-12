//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/trigger_change_ui.h"

namespace rm_referee
{
void TriggerChangeUi::setContent(const std::string& content)
{
  graph_->setContent(content);
  display();
}

void TriggerChangeUi::display()
{
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display();
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::display()
{
  if (s_l_ == rm_msgs::DbusData::MID && s_r_ == rm_msgs::DbusData::UP)
    updateConfig(chassis_mode_, false, 1, false);
  else
    updateConfig(chassis_mode_, power_limit_state_ == rm_common::PowerLimit::BURST, 0,
                 power_limit_state_ == rm_common::PowerLimit::CHARGE);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::displayInCapacity()
{
  if (key_ctrl_ && key_shift_ && key_b_ && base_.robot_id_ != rm_referee::RobotId::RED_ENGINEER &&
      base_.robot_id_ != rm_referee::RobotId::BLUE_ENGINEER)
    updateConfig(254, 0);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  if (main_mode == 254)
  {
    graph_->setContent("Cap reset");
    graph_->setColor(rm_referee::GraphColor::YELLOW);
    return;
  }
  graph_->setContent(getChassisState(main_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    graph_->setColor(rm_referee::GraphColor::GREEN);
  else if (sub_mode == 1)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string ChassisTriggerChangeUi::getChassisState(uint8_t mode)
{
  if (mode == rm_msgs::ChassisCmd::RAW)
    return "raw";
  else if (mode == rm_msgs::ChassisCmd::FOLLOW)
    return "follow";
  else if (mode == rm_msgs::ChassisCmd::TWIST)
    return "twist";
  else
    return "error";
}

void ChassisTriggerChangeUi::updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data)
{
  chassis_mode_ = data->mode;
  display();
}

void ChassisTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  power_limit_state_ = data->power_limit_state;
}

void ChassisTriggerChangeUi::updateDbusData(const rm_msgs::DbusData::ConstPtr data)
{
  s_l_ = data->s_l;
  s_r_ = data->s_r;
  key_ctrl_ = data->key_ctrl;
  key_shift_ = data->key_shift;
  key_b_ = data->key_b;
}

void ChassisTriggerChangeUi::updateCapacityData(const rm_msgs::CapacityData data)
{
  displayInCapacity();
}

void ShooterTriggerChangeUi::display()
{
  updateConfig(shooter_mode_, 0, shoot_frequency_, false);
  TriggerChangeUi::display();
}

void ShooterTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getShooterState(main_mode));
  if (sub_mode == rm_common::HeatLimit::LOW)
    graph_->setColor(rm_referee::GraphColor::WHITE);
  else if (sub_mode == rm_common::HeatLimit::HIGH)
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  else if (sub_mode == rm_common::HeatLimit::BURST)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
}

std::string ShooterTriggerChangeUi::getShooterState(uint8_t state)
{
  if (state == rm_msgs::ShootState::STOP)
    return "stop";
  else if (state == rm_msgs::ShootState::READY)
    return "ready";
  else if (state == rm_msgs::ShootState::PUSH)
    return "push";
  else if (state == rm_msgs::ShootState::BLOCK)
    return "block";
  else
    return "error";
}

void ShooterTriggerChangeUi::updateShootStateData(const rm_msgs::ShootState::ConstPtr& data)
{
  shooter_mode_ = data->state;
  display();
}

void ShooterTriggerChangeUi::updateManualCmdData(rm_msgs::ManualToReferee::ConstPtr data)
{
  shoot_frequency_ = data->shoot_frequency;
}

void GimbalTriggerChangeUi::display()
{
  updateConfig(gimbal_mode_, gimbal_eject_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void GimbalTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getGimbalState(main_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string GimbalTriggerChangeUi::getGimbalState(uint8_t mode)
{
  if (mode == rm_msgs::GimbalCmd::DIRECT)
    return "direct";
  else if (mode == rm_msgs::GimbalCmd::RATE)
    return "rate";
  else if (mode == rm_msgs::GimbalCmd::TRACK)
    return "track";
  else
    return "error";
}

void GimbalTriggerChangeUi::updateGimbalCmdData(const rm_msgs::GimbalCmd::ConstPtr data)
{
  gimbal_mode_ = data->mode;
  display();
}

void GimbalTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  gimbal_eject_ = data->gimbal_eject;
}

void TargetTriggerChangeUi::display()
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
    updateConfig(det_target_, shoot_frequency_ == rm_common::HeatLimit::BURST, det_armor_target_,
                 det_color_ == rm_msgs::StatusChangeRequest::RED);
  else
    updateConfig(gimbal_eject_, shoot_frequency_, det_armor_target_, det_color_ == rm_msgs::StatusChangeRequest::RED);
  TriggerChangeUi::display();
}

void TargetTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getTargetState(main_mode, sub_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::CYAN);
}

std::string TargetTriggerChangeUi::getTargetState(uint8_t target, uint8_t armor_target)
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
  {
    if (target == rm_msgs::StatusChangeRequest::BUFF)
      return "buff";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR && armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "armor_all";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR &&
             armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "armor_base";
    else
      return "error";
  }
  else
  {
    if (target == 1)
      return "eject";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "all";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "base";
    else
      return "error";
  }
}

void TargetTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  det_target_ = data->det_target;
  shoot_frequency_ = data->shoot_frequency;
  det_armor_target_ = data->det_armor_target;
  det_color_ = data->det_color;
  gimbal_eject_ = data->gimbal_eject;
}

void TargetTriggerChangeUi::updateShootStateData(const rm_msgs::ShootState::ConstPtr& data)
{
  display();
}

void PolygonTriggerChangeGroupUi::display()
{
  for (auto graph : graph_vector_)
  {
    graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
    graph.second->display();
    graph.second->sendUi(ros::Time::now());
  }
}

void CameraTriggerChangeUi::updateCameraName(const std_msgs::StringConstPtr& data)
{
  current_camera_ = data->data;
  display();
}

void CameraTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(current_camera_);
  if (current_camera_ == camera1_name_)
    graph_->setColor(rm_referee::GraphColor::CYAN);
  else if (current_camera_ == camera2_name_)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}
void CameraTriggerChangeUi::display()
{
  updateConfig();
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  TriggerChangeUi::display();
  graph_->sendUi(ros::Time::now());
}
void DragTriggerChangeUi::updateDragUiData(const rm_msgs::EngineerUi ::ConstPtr& data)
{
  drag_state_ = data->drag_state;
  display();
}

void DragTriggerChangeUi::display()
{
  dragUpdateConfig(drag_state_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

void DragTriggerChangeUi::dragUpdateConfig(const std::string& drag_state)
{
  graph_->setContent(drag_state);
  graph_->setColor(rm_referee::GraphColor::GREEN);
}

void GripperTriggerChangeUi::updateGripperUiData(const rm_msgs::EngineerUi ::ConstPtr& data)
{
  gripper_state_ = data->gripper_state;
  display();
}

void GripperTriggerChangeUi::display()
{
  gripperUpdateConfig(gripper_state_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

void GripperTriggerChangeUi::gripperUpdateConfig(const std::string& gripper_state)
{
  graph_->setContent(gripper_state);
  graph_->setColor(rm_referee::GraphColor::GREEN);
}

void ExchangeTriggerChangeUi::updateExchangeData(const rm_msgs::ExchangerMsg::ConstPtr& data)
{
  exchange_state_.shape = data->shape;
  exchange_state_.flag = data->shape;
  display();
}

void ExchangeTriggerChangeUi::display()
{
  exchangeUpdateConfig(exchange_state_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display(true);
  graph_->sendUi(ros::Time::now());
}

void ExchangeTriggerChangeUi::exchangeUpdateConfig(const rm_msgs::ExchangerMsg& exchange_state)
{
  if (exchange_state.flag)
    graph_->setContent("CAN!");
  else if (!exchange_state.flag)
    graph_->setContent("CAN NOT");
  if (exchange_state.shape)
    graph_->setColor(rm_referee::GraphColor::GREEN);
  else if (!exchange_state.shape)
    graph_->setColor(rm_referee::GraphColor::YELLOW);
}

void PlanningResultTriggerChangeUi::updatePlanningResultData(const std_msgs::Int32::ConstPtr& data)
{
  planning_result_.data = data->data;
  display();
}

void PlanningResultTriggerChangeUi::display()
{
  planningResultUpdateConfig(planning_result_);
  {
    display();
  }

  void PolygonTriggerChangeGroupUi::display()
  {
    for (auto graph : graph_vector_)
    {
      graph.second->setOperation(rm_referee::GraphOperation::UPDATE);
      graph.second->display();
      graph.second->sendUi(ros::Time::now());
    }
  }

  void CameraTriggerChangeUi::updateCameraName(const std_msgs::StringConstPtr& data)
  {
    current_camera_ = data->data;
    display();
  }

  void CameraTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
  {
    graph_->setContent(current_camera_);
    if (current_camera_ == camera1_name_)
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else if (current_camera_ == camera2_name_)
      graph_->setColor(rm_referee::GraphColor::ORANGE);
    else
      graph_->setColor(rm_referee::GraphColor::WHITE);
  }
  void CameraTriggerChangeUi::display()
  {
    updateConfig();
    >>>>>>> master graph_->setOperation(rm_referee::GraphOperation::UPDATE);
    TriggerChangeUi::display();
    graph_->sendUi(ros::Time::now());
  }

  void PlanningResultTriggerChangeUi::planningResultUpdateConfig(const std_msgs::Int32& planning_result)
  {
    graph_->setColor(rm_referee::GraphColor::YELLOW);
    if (planning_result.data == 1)
    {
      graph_->setContent("SUCCESS");
      graph_->setColor(rm_referee::GraphColor::GREEN);
    }
    else if (planning_result.data == 99999)
      graph_->setContent("FAILURE");
    else if (planning_result.data == -1)
      graph_->setContent("PLANNING_FAILED");
    else if (planning_result.data == -2)
      graph_->setContent("INVALID_MOTION_PLAN");
    else if (planning_result.data == -3)
      graph_->setContent("MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
    else if (planning_result.data == -4)
      graph_->setContent("CONTROL_FAILED");
    else if (planning_result.data == -5)
      graph_->setContent("UNABLE_TO_AQUIRE_SENSOR_DATA");
    else if (planning_result.data == -6)
      graph_->setContent("TIMED_OUT");
    else if (planning_result.data == -7)
      graph_->setContent("PREEMPTED");
    else if (planning_result.data == -10)
      graph_->setContent("START_STATE_IN_COLLISION");
    else if (planning_result.data == -11)
      graph_->setContent("START_STATE_VIOLATES_PATH_CONSTRAINTS");
    else if (planning_result.data == -12)
      graph_->setContent("GOAL_IN_COLLISION");
    else if (planning_result.data == -13)
      graph_->setContent("GOAL_VIOLATES_PATH_CONSTRAINTS");
    else if (planning_result.data == -14)
      graph_->setContent("GOAL_CONSTRAINTS_VIOLATED");
    else if (planning_result.data == -15)
      graph_->setContent("INVALID_GROUP_NAME");
    else if (planning_result.data == -16)
      graph_->setContent("INVALID_GOAL_CONSTRAINTS");
    else if (planning_result.data == -17)
      graph_->setContent("INVALID_ROBOT_STATE");
    else if (planning_result.data == -18)
      graph_->setContent("INVALID_LINK_NAME");
    else if (planning_result.data == -19)
      graph_->setContent("INVALID_OBJECT_NAME");
    else if (planning_result.data == -21)
      graph_->setContent("FRAME_TRANSFORM_FAILURE");
    else if (planning_result.data == -22)
      graph_->setContent("COLLISION_CHECKING_UNAVAILABLE");
    else if (planning_result.data == -23)
      graph_->setContent("ROBOT_STATE_STALE");
    else if (planning_result.data == -24)
      graph_->setContent("SENSOR_INFO_STALE");
    else if (planning_result.data == -31)
      graph_->setContent("NO_IK_SOLUTION");
  }

  void StepTriggerChangeUi::updateStepUiData(const rm_msgs::EngineerUi ::ConstPtr data)
  {
    step_name_ = data->current_step_name;
    display();
  }

  void StepTriggerChangeUi::display()
  {
    graph_->setContent(step_name_);
    graph_->setColor(rm_referee::GraphColor::GREEN);
    graph_->setOperation(rm_referee::GraphOperation::UPDATE);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }

  void ReversalTriggerChangeUi::updateReversalUiData(const rm_msgs::EngineerUi ::ConstPtr data)
  {
    reversal_state_ = data->reversal_state;
    display();
  }

  void ReversalTriggerChangeUi::display()
  {
    graph_->setContent(reversal_state_);
    graph_->setColor(rm_referee::GraphColor::GREEN);
    graph_->setOperation(rm_referee::GraphOperation::UPDATE);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }

  void StoneTriggerChangeUi::updateStoneUiData(const rm_msgs::EngineerUi ::ConstPtr data)
  {
    stone_num_ = data->stone_num;
    display();
  }

  void StoneTriggerChangeUi::display()
  {
    stoneUpdateConfig(stone_num_);
    graph_->setOperation(rm_referee::GraphOperation::UPDATE);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }

  void StoneTriggerChangeUi::stoneUpdateConfig(uint8_t stone_num)
  {
    graph_->setContent(getStoneNum(stone_num));
    graph_->setColor(rm_referee::GraphColor::GREEN);
  }

  std::string StoneTriggerChangeUi::getStoneNum(uint8_t stone_num)
  {
    if (!stone_num)
      return "0";
    else if (stone_num == 1)
      return "1";
    else if (stone_num == 2)
      return "2";
    else
      return "3";
  }

  void JointTemperatureTriggerChangeUi::updateJointTemperatureUiData(const rm_msgs::EngineerUi ::ConstPtr data)
  {
    joint_temperature_ = data->joint_temperature;
    display();
  }

  void JointTemperatureTriggerChangeUi::display()
  {
    graph_->setContent(joint_temperature_);
    graph_->setColor(rm_referee::GraphColor::GREEN);
    graph_->setOperation(rm_referee::GraphOperation::UPDATE);
    graph_->display(true);
    graph_->sendUi(ros::Time::now());
  }

}  // namespace rm_referee
