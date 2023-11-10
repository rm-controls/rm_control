//
// Created by ljq on 2022/5/17.
//

#include "rm_referee/referee_base.h"

namespace rm_referee
{
RefereeBase::RefereeBase(ros::NodeHandle& nh, Base& base) : base_(base), nh_(nh)
{
  RefereeBase::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &RefereeBase::jointStateCallback, this);
  RefereeBase::actuator_state_sub_ =
      nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &RefereeBase::actuatorStateCallback, this);
  RefereeBase::dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &RefereeBase::dbusDataCallback, this);
  RefereeBase::chassis_cmd_sub_ =
      nh.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 10, &RefereeBase::chassisCmdDataCallback, this);
  RefereeBase::vel2D_cmd_sub_ =
      nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RefereeBase::vel2DCmdDataCallback, this);
  RefereeBase::shoot_state_sub_ = nh.subscribe<rm_msgs::ShootState>("/controllers/shooter_controller/state", 10,
                                                                    &RefereeBase::shootStateCallback, this);
  RefereeBase::gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                                  &RefereeBase::gimbalCmdDataCallback, this);
  RefereeBase::card_cmd_sub_ = nh.subscribe<rm_msgs::StateCmd>("/controllers/card_controller/command", 10,
                                                               &RefereeBase::cardCmdDataCallback, this);
  RefereeBase::engineer_cmd_sub_ =
      nh.subscribe<rm_msgs::EngineerUi>("/engineer_ui", 10, &RefereeBase::engineerUiDataCallback, this);
  RefereeBase::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &RefereeBase::manualDataCallBack, this);
  RefereeBase::camera_name_sub_ = nh.subscribe("/camera_name", 10, &RefereeBase::cameraNameCallBack, this);
  RefereeBase::balance_state_sub_ = nh.subscribe("/state", 10, &RefereeBase::balanceStateCallback, this);
  RefereeBase::track_sub_ = nh.subscribe<rm_msgs::TrackData>("/track", 10, &RefereeBase::trackCallBack, this);
  RefereeBase::map_sentry_sub_ =
      nh.subscribe<rm_msgs::MapSentryData>("/map_sentry_data", 10, &RefereeBase::mapSentryCallback, this);
  RefereeBase::radar_receive_sub_ =
      nh.subscribe<rm_msgs::ClientMapReceiveData>("/rm_radar", 10, &RefereeBase::radarReceiveCallback, this);
  RefereeBase::sentry_deviate_sub_ =
      nh.subscribe<rm_msgs::SentryDeviate>("/deviate", 10, &RefereeBase::sentryDeviateCallback, this);
  RefereeBase::radar_to_sentry_sub_ = nh.subscribe<rm_msgs::CurrentSentryPosData>(
      "/radar_to_sentry", 10, &RefereeBase::sendCurrentSentryCallback, this);

  XmlRpc::XmlRpcValue rpc_value;
  send_ui_queue_delay_ = getParam(nh, "send_ui_queue_delay", 0.15);
  add_ui_frequency_ = getParam(nh, "add_ui_frequency", 5);
  add_ui_max_times_ = getParam(nh, "add_ui_max_times", 10);
  interactive_data_sender_ = new UiBase(rpc_value, base_);
  if (nh.hasParam("ui"))
  {
    ros::NodeHandle ui_nh(nh, "ui");
    graph_queue_sender_ = new GroupUiBase(rpc_value, base_);
    ui_nh.getParam("trigger_change", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "chassis")
        chassis_trigger_change_ui_ = new ChassisTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "shooter")
        shooter_trigger_change_ui_ = new ShooterTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "gimbal")
        gimbal_trigger_change_ui_ = new GimbalTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "target")
        target_trigger_change_ui_ = new TargetTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "target_view_angle")
        target_view_angle_trigger_change_ui_ = new TargetViewAngleTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "camera")
        camera_trigger_change_ui_ = new CameraTriggerChangeUi(rpc_value[i], base_, &graph_queue_);
    }

    ui_nh.getParam("time_change", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "capacitor")
        capacitor_time_change_ui_ = new CapacitorTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "effort")
        effort_time_change_ui_ = new EffortTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "progress")
        progress_time_change_ui_ = new ProgressTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "dart_status")
        dart_status_time_change_ui_ = new DartStatusTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "rotation")
        rotation_time_change_ui_ = new RotationTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "lane_line")
        lane_line_time_change_ui_ = new LaneLineTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "pitch")
        pitch_angle_time_change_ui_ = new PitchAngleTimeChangeUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "balance_pitch")
        balance_pitch_time_change_group_ui_ = new BalancePitchTimeChangeGroupUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "engineer_joint1")
        engineer_joint1_time_change_ui = new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, "joint1");
      if (rpc_value[i]["name"] == "engineer_joint2")
        engineer_joint2_time_change_ui = new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, "joint2");
      if (rpc_value[i]["name"] == "engineer_joint3")
        engineer_joint3_time_change_ui = new JointPositionTimeChangeUi(rpc_value[i], base_, &graph_queue_, "joint3");
    }

    ui_nh.getParam("fixed", rpc_value);
    fixed_ui_ = new FixedUi(rpc_value, base_, &graph_queue_);

    ui_nh.getParam("flash", rpc_value);
    for (int i = 0; i < rpc_value.size(); i++)
    {
      if (rpc_value[i]["name"] == "cover")
        cover_flash_ui_ = new CoverFlashUi(rpc_value[i], base_, &graph_queue_);
      if (rpc_value[i]["name"] == "spin")
        spin_flash_ui_ = new SpinFlashUi(rpc_value[i], base_, &graph_queue_);
    }
  }

  add_ui_timer_ =
      nh.createTimer(ros::Duration(1. / add_ui_frequency_), std::bind(&RefereeBase::addUi, this), false, false);
  send_graph_ui_timer_ = nh.createTimer(ros::Duration(send_ui_queue_delay_),
                                        std::bind(&RefereeBase::sendGraphQueueCallback, this), false, true);
}
void RefereeBase::addUi()
{
  if (add_ui_times_ > add_ui_max_times_)
  {
    ROS_INFO("End add");
    add_ui_timer_.stop();
    if (!graph_queue_.empty())
      while (graph_queue_.size() > 0)
        graph_queue_.pop_back();
    is_adding_ = false;
    send_graph_ui_timer_.setPeriod(ros::Duration(send_ui_queue_delay_));
    return;
  }

  ROS_INFO_THROTTLE(1.0, "Adding ui... %.1f%%", (add_ui_times_ / static_cast<double>(add_ui_max_times_)) * 100);
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->addForQueue(3);
  if (gimbal_trigger_change_ui_)
    gimbal_trigger_change_ui_->addForQueue(3);
  if (shooter_trigger_change_ui_)
    shooter_trigger_change_ui_->addForQueue(3);
  if (target_trigger_change_ui_)
    target_trigger_change_ui_->addForQueue();
  if (target_view_angle_trigger_change_ui_)
    target_view_angle_trigger_change_ui_->addForQueue();
  if (camera_trigger_change_ui_)
    camera_trigger_change_ui_->addForQueue();
  if (fixed_ui_)
    fixed_ui_->addForQueue();
  if (effort_time_change_ui_)
    effort_time_change_ui_->addForQueue();
  if (progress_time_change_ui_)
    progress_time_change_ui_->addForQueue();
  if (dart_status_time_change_ui_)
    dart_status_time_change_ui_->addForQueue();
  if (capacitor_time_change_ui_)
    capacitor_time_change_ui_->addForQueue();
  if (rotation_time_change_ui_)
    rotation_time_change_ui_->addForQueue();
  if (lane_line_time_change_ui_)
    lane_line_time_change_ui_->addForQueue();
  if (balance_pitch_time_change_group_ui_)
    balance_pitch_time_change_group_ui_->addForQueue();
  if (pitch_angle_time_change_ui_)
    pitch_angle_time_change_ui_->addForQueue();
  if (engineer_joint1_time_change_ui)
    engineer_joint1_time_change_ui->addForQueue();
  if (engineer_joint2_time_change_ui)
    engineer_joint2_time_change_ui->addForQueue();
  if (engineer_joint3_time_change_ui)
    engineer_joint3_time_change_ui->addForQueue();
  add_ui_times_++;
}

void RefereeBase::sendGraphQueueCallback()
{
  if (graph_queue_.empty())
    return;

  if (graph_queue_.size() > 50)
  {
    ROS_WARN_THROTTLE(2.0, "Sending UI too frequently, please modify the configuration file or code to "
                           "reduce the frequency");
    while (graph_queue_.size() > 50)
      graph_queue_.pop_back();
  }

  int index = graph_queue_.size() - 1;

  if (!is_adding_)
  {
    if (graph_queue_.size() >= 7)
    {
      graph_queue_sender_->sendSevenGraph(ros::Time::now(), &graph_queue_.at(index), &graph_queue_.at(index - 1),
                                          &graph_queue_.at(index - 2), &graph_queue_.at(index - 3),
                                          &graph_queue_.at(index - 4), &graph_queue_.at(index - 5),
                                          &graph_queue_.at(index - 6));
      for (int i = 0; i < 7; i++)
        graph_queue_.pop_back();
    }
    else if (graph_queue_.size() >= 5)
    {
      graph_queue_sender_->sendFiveGraph(ros::Time::now(), &graph_queue_.at(index), &graph_queue_.at(index - 1),
                                         &graph_queue_.at(index - 2), &graph_queue_.at(index - 3),
                                         &graph_queue_.at(index - 4));
      for (int i = 0; i < 5; i++)
        graph_queue_.pop_back();
    }
    else if (graph_queue_.size() >= 2)
    {
      graph_queue_sender_->sendDoubleGraph(ros::Time::now(), &graph_queue_.at(index), &graph_queue_.at(index - 1));
      for (int i = 0; i < 2; i++)
        graph_queue_.pop_back();
    }
    else if (graph_queue_.size() == 1)
    {
      graph_queue_sender_->sendSingleGraph(ros::Time::now(), &graph_queue_.at(index));
      graph_queue_.pop_back();
    }
  }
  else
  {
    graph_queue_sender_->sendSingleGraph(ros::Time::now(), &graph_queue_.at(index));
    graph_queue_.pop_back();
  }

  send_graph_ui_timer_.start();
}

void RefereeBase::robotStatusDataCallBack(const rm_msgs::GameRobotStatus& data, const ros::Time& last_get_data_time)
{
  if (fixed_ui_ && !is_adding_)
    fixed_ui_->update();
}
void RefereeBase::gameStatusDataCallBack(const rm_msgs::GameStatus& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::capacityDataCallBack(const rm_msgs::PowerManagementSampleAndStatusData& data,
                                       ros::Time& last_get_data_time)
{
  if (capacitor_time_change_ui_ && !is_adding_)
    capacitor_time_change_ui_->updateRemainCharge(data.capacity_remain_charge, last_get_data_time);
  if (chassis_trigger_change_ui_ && !is_adding_)
    chassis_trigger_change_ui_->updateCapacityResetStatus();
}
void RefereeBase::powerHeatDataCallBack(const rm_msgs::PowerHeatData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::robotHurtDataCallBack(const rm_msgs::RobotHurt& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::interactiveDataCallBack(const rm_referee::InteractiveData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::eventDataCallBack(const rm_msgs::EventData& data, const ros::Time& last_get_data_time)
{
}
void RefereeBase::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  if (effort_time_change_ui_ && !is_adding_)
    effort_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (rotation_time_change_ui_ && !is_adding_)
    rotation_time_change_ui_->updateForQueue();
  if (lane_line_time_change_ui_ && !is_adding_)
    lane_line_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (pitch_angle_time_change_ui_ && !is_adding_)
    pitch_angle_time_change_ui_->updateJointStateData(data, ros::Time::now());
  if (engineer_joint1_time_change_ui && !is_adding_)
    engineer_joint1_time_change_ui->updateJointStateData(data, ros::Time::now());
  if (engineer_joint2_time_change_ui && !is_adding_)
    engineer_joint2_time_change_ui->updateJointStateData(data, ros::Time::now());
  if (engineer_joint3_time_change_ui && !is_adding_)
    engineer_joint3_time_change_ui->updateJointStateData(data, ros::Time::now());
}
void RefereeBase::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
}
void RefereeBase::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  if (add_ui_flag_ && data->s_r == rm_msgs::DbusData::UP)
  {
    add_ui_flag_ = false;
    is_adding_ = true;
    if (!graph_queue_.empty())
      while (graph_queue_.size() > 0)
        graph_queue_.pop_back();
    send_graph_ui_timer_.setPeriod(ros::Duration(0.05));
    add_ui_timer_.start();
    add_ui_times_ = 0;
  }
  if (data->s_r != rm_msgs::DbusData::UP)
  {
    add_ui_flag_ = true;
    add_ui_timer_.stop();
  }
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateDbusData(data);
}
void RefereeBase::chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateChassisCmdData(data);
  if (spin_flash_ui_ && !is_adding_)
    spin_flash_ui_->updateChassisCmdData(data, ros::Time::now());
}
void RefereeBase::vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
{
}
void RefereeBase::shootStateCallback(const rm_msgs::ShootState::ConstPtr& data)
{
  if (target_trigger_change_ui_ && !is_adding_)
    target_trigger_change_ui_->updateShootStateData(data);
  if (shooter_trigger_change_ui_ && !is_adding_)
    shooter_trigger_change_ui_->updateShootStateData(data);
}
void RefereeBase::gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
{
  if (gimbal_trigger_change_ui_ && !is_adding_)
    gimbal_trigger_change_ui_->updateGimbalCmdData(data);
}
void RefereeBase::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
}
void RefereeBase::engineerUiDataCallback(const rm_msgs::EngineerUi::ConstPtr& data)
{
  if (progress_time_change_ui_ && !is_adding_)
    progress_time_change_ui_->updateEngineerUiData(data, ros::Time::now());
}
void RefereeBase::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  if (chassis_trigger_change_ui_)
    chassis_trigger_change_ui_->updateManualCmdData(data);
  if (shooter_trigger_change_ui_ && !is_adding_)
    shooter_trigger_change_ui_->updateManualCmdData(data);
  if (gimbal_trigger_change_ui_ && !is_adding_)
    gimbal_trigger_change_ui_->updateManualCmdData(data);
  if (target_trigger_change_ui_ && !is_adding_)
    target_trigger_change_ui_->updateManualCmdData(data);
  if (cover_flash_ui_ && !is_adding_)
    cover_flash_ui_->updateManualCmdData(data, ros::Time::now());
}
void RefereeBase::radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data)
{
}
void RefereeBase::cameraNameCallBack(const std_msgs::StringConstPtr& data)
{
  if (camera_trigger_change_ui_ && !is_adding_)
    camera_trigger_change_ui_->updateCameraName(data);
}
void RefereeBase::trackCallBack(const rm_msgs::TrackDataConstPtr& data)
{
  if (target_view_angle_trigger_change_ui_ && !is_adding_)
    target_view_angle_trigger_change_ui_->updateTrackID(data->id);
}
void RefereeBase::balanceStateCallback(const rm_msgs::BalanceStateConstPtr& data)
{
  if (balance_pitch_time_change_group_ui_)
    balance_pitch_time_change_group_ui_->calculatePointPosition(data, ros::Time::now());
}
void RefereeBase::sentryDeviateCallback(const rm_msgs::SentryDeviateConstPtr& data)
{
}
void RefereeBase::radarReceiveCallback(const rm_msgs::ClientMapReceiveData::ConstPtr& data)
{
  rm_referee::ClientMapReceiveData send_data;
  send_data.target_position_x = data->target_position_x;
  send_data.target_position_y = data->target_position_y;
  send_data.target_robot_ID = data->target_robot_ID;

  interactive_data_sender_->sendRadarInteractiveData(send_data);
}
void RefereeBase::mapSentryCallback(const rm_msgs::MapSentryDataConstPtr& data)
{
  interactive_data_sender_->sendMapSentryData(data);
}

void RefereeBase::sendCurrentSentryCallback(const rm_msgs::CurrentSentryPosDataConstPtr& data)
{
  interactive_data_sender_->sendCurrentSentryData(data);
}

}  // namespace rm_referee
