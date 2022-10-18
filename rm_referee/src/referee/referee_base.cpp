//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee/referee_base.h"

namespace rm_referee
{
RefereeBase::RefereeBase(ros::NodeHandle& nh, Base& base) : base_(base), nh_(nh)
{
  RefereeBase::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &RefereeBase::jointStateCallback, this);
  RefereeBase::actuator_state_sub_ =
      nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &RefereeBase::actuatorStateCallback, this);
  RefereeBase::dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &RefereeBase::dbusDataCallback, this);
  RefereeBase::chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                                    &RefereeBase::chassisCmdDataCallback, this);
  RefereeBase::vel2D_cmd_sub_ =
      nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RefereeBase::vel2DCmdDataCallback, this);
  RefereeBase::shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                                &RefereeBase::shootCmdDataCallback, this);
  RefereeBase::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &RefereeBase::gimbalCmdDataCallback, this);
  RefereeBase::card_cmd_sub_ = nh.subscribe<rm_msgs::StateCmd>("/controllers/card_controller/command", 10,
                                                               &RefereeBase::cardCmdDataCallback, this);
  RefereeBase::engineer_cmd_sub_ =
      nh.subscribe<rm_msgs::EngineerCmd>("/engineer_cmd", 10, &RefereeBase::engineerCmdDataCallback, this);
  RefereeBase::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &RefereeBase::manualDataCallBack, this);
  if (base_.robot_id_ == rm_referee::RobotId::RED_RADAR || base_.robot_id_ == rm_referee::RobotId::BLUE_RADAR)
    RefereeBase::radar_date_sub_ =
        nh.subscribe<std_msgs::Int8MultiArray>("/data", 10, &RefereeBase::radarDataCallBack, this);
  XmlRpc::XmlRpcValue rpc_value;
  ros::NodeHandle ui_nh(nh, "ui");
  ui_nh.getParam("trigger_change", rpc_value);
  for (int i = 0; i < rpc_value.size(); i++)
  {
    if (rpc_value[i]["name"] == "chassis")
      chassis_trigger_change_ui_ = new ChassisTriggerChangeUi(ui_nh, base_);
    if (rpc_value[i]["name"] == "shooter")
      shooter_trigger_change_ui_ = new ShooterTriggerChangeUi(ui_nh, base_);
    if (rpc_value[i]["name"] == "gimbal")
      gimbal_trigger_change_ui_ = new GimbalTriggerChangeUi(ui_nh, base_);
    if (rpc_value[i]["name"] == "target")
      target_trigger_change_ui_ = new TargetTriggerChangeUi(ui_nh, base_);
  }

  ui_nh.getParam("time_change", rpc_value);
  for (int i = 0; i < rpc_value.size(); i++)
  {
    if (rpc_value[i]["name"] == "capacitor")
      capacitor_time_change_ui_ = new CapacitorTimeChangeUI(ui_nh, base_);
    if (rpc_value[i]["name"] == "effort")
      effort_time_change_ui_ = new EffortTimeChangeUI(ui_nh, base_);
    if (rpc_value[i]["name"] == "progress")
      progress_time_change_ui_ = new ProgressTimeChangeUI(ui_nh, base_);
    if (rpc_value[i]["name"] == "dart_status")
      dart_status_time_change_ui_ = new DartStatusTimeChangeUI(ui_nh, base_);
    if (rpc_value[i]["name"] == "ore_remind")
      ore_remind_time_change_ui_ = new OreRemindTimeChangeUI(ui_nh, base_);
  }

  fixed_ui_ = new FixedUi(ui_nh, base_);

  ui_nh.getParam("flash", rpc_value);
  for (int i = 0; i < rpc_value.size(); i++)
    if (rpc_value[i]["name"] == "cover")
      cover_flash_ui_ = new CoverFlashUI(ui_nh, base_);
}
void RefereeBase::addUi()
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->add();
  if (gimbal_trigger_change_ui_)
    gimbal_trigger_change_ui_->add();
  if (target_trigger_change_ui_)
    target_trigger_change_ui_->add();
  if (capacitor_time_change_ui_)
    capacitor_time_change_ui_->add();
  if (exposure_trigger_change_ui_)
    exposure_trigger_change_ui_->add();

  if (fixed_ui_)
    fixed_ui_->add();

  if (effort_time_change_ui_)
    effort_time_change_ui_->add();
  if (progress_time_change_ui_)
    progress_time_change_ui_->add();
  if (dart_status_time_change_ui_)
    dart_status_time_change_ui_->add();
  if (ore_remind_time_change_ui_)
    ore_remind_time_change_ui_->add();
}

void RefereeBase::run()
{
}

void RefereeBase::robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data_,
                                          const ros::Time& last_get_)
{
  if (fixed_ui_)
    fixed_ui_->update();
}
void RefereeBase::gameStatusDataCallBack(const rm_msgs::GameStatus& game_status_data_, const ros::Time& last_get_)
{
}
void RefereeBase::capacityDataCallBack(const rm_msgs::CapacityData& capacity_data_, ros::Time& last_get_)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->capacityDataCallBack();
  if (capacitor_time_change_ui_)
    capacitor_time_change_ui_->capPowerDataCallBack(capacity_data_.cap_power, last_get_);
}
void RefereeBase::powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data_, const ros::Time& last_get_)
{
}
void RefereeBase::robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data_, const ros::Time& last_get_)
{
}
void RefereeBase::interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data_,
                                          const ros::Time& last_get_)
{
}
void RefereeBase::eventDataCallBack(const rm_msgs::EventData& event_data_, const ros::Time& last_get_)
{
}
void RefereeBase::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  if (effort_time_change_ui_)
  {
    int max_index = 0;
    {
      if (!data->name.empty())
      {
        for (int i = 0; i < static_cast<int>(data->effort.size()); ++i)
          if ((data->name[i] == "joint1" || data->name[i] == "joint2" || data->name[i] == "joint3" ||
               data->name[i] == "joint4" || data->name[i] == "joint5") &&
              data->effort[i] > data->effort[max_index])
            max_index = i;
      }
    }
    if (max_index != 0)
    {
      effort_time_change_ui_->EffortDataCallBack(data->name[max_index], data->effort[max_index]);
    }
  }
}
void RefereeBase::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
}
void RefereeBase::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->DbusDataCallBack(data->s_l, data->s_r, data->key_ctrl, data->key_shift, data->key_b);
  if (data->s_r == rm_msgs::DbusData::UP)
    send_ui_flag_ = true;
}
void RefereeBase::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->ChassisModeCallBack(data->mode);
}
void RefereeBase::vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
{
}
void RefereeBase::shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
{
  if (shooter_trigger_change_ui_)
    shooter_trigger_change_ui_->ShooterModeCallBack(data->mode);
  if (target_trigger_change_ui_)
    target_trigger_change_ui_->ShooterCallBack();
}
void RefereeBase::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  if (gimbal_trigger_change_ui_)
    gimbal_trigger_change_ui_->GimbalModeCallback(data->mode);
}
void RefereeBase::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
}
void RefereeBase::engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data)
{
  if (progress_time_change_ui_)
    progress_time_change_ui_->progressDataCallBack(data->finished_step, data->total_steps, data->step_queue_name);
}
void RefereeBase::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->PowerLimitStateCallBack(data->power_limit_state);
  if (shooter_trigger_change_ui_)
    shooter_trigger_change_ui_->ShootFrequencyCallBack(data->shoot_frequency);
  if (gimbal_trigger_change_ui_)
    gimbal_trigger_change_ui_->GimbalEjectCallBack(data->gimbal_eject);
  if (target_trigger_change_ui_)
    target_trigger_change_ui_->ManalToRefereeCallback(data->det_target, data->shoot_frequency, data->det_armor_target,
                                                      data->det_color, data->gimbal_eject);
  if (cover_flash_ui_)
    cover_flash_ui_->coverStateCallBack(data->cover_state);
}
void RefereeBase::radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data)
{
}

}  // namespace rm_referee
