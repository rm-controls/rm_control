//
// Created by ljq on 2021/12/3.
//

#pragma once

#include <rm_common/ros_utilities.h>
#include <ros/timer.h>
#include <rm_common/decision/command_sender.h>

#include "rm_referee/ui/ui_base.h"
#include "rm_referee/ui/trigger_change_ui.h"
#include "rm_referee/ui/time_change_ui.h"
#include "rm_referee/ui/flash_ui.h"

namespace rm_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(ros::NodeHandle& nh, Base& base);
  virtual void addUi();

  // unpack call back
  virtual void robotStatusDataCallBack(const rm_msgs::GameRobotStatus& game_robot_status_data,
                                       const ros::Time& last_get_data_time);
  virtual void updateEnemyHeroState(const rm_msgs::GameRobotHp& game_robot_hp_data, const ros::Time& last_get_data_time);
  virtual void gameStatusDataCallBack(const rm_msgs::GameStatus& game_status_data, const ros::Time& last_get_data_time);
  virtual void capacityDataCallBack(const rm_msgs::PowerManagementSampleAndStatusData& data,
                                    ros::Time& last_get_data_time);
  virtual void powerHeatDataCallBack(const rm_msgs::PowerHeatData& power_heat_data, const ros::Time& last_get_data_time);
  virtual void robotHurtDataCallBack(const rm_msgs::RobotHurt& robot_hurt_data, const ros::Time& last_get_data_time);
  virtual void bulletRemainDataCallBack(const rm_msgs::BulletAllowance& bullet_allowance,
                                        const ros::Time& last_get_data_time);
  virtual void interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data,
                                       const ros::Time& last_get_data_time);
  virtual void eventDataCallBack(const rm_msgs::EventData& event_data, const ros::Time& last_get_data_time);
  virtual void updateHeroHitDataCallBack(const rm_msgs::GameRobotHp& game_robot_hp_data);
  virtual void supplyBulletDataCallBack(const rm_msgs::SupplyProjectileAction& data);
  virtual void updateShootDataDataCallBack(const rm_msgs::ShootData& msg);

  // sub call back
  virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data);
  virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data);
  virtual void chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data);
  virtual void vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data);
  virtual void shootStateCallback(const rm_msgs::ShootState::ConstPtr& data);
  virtual void gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data);
  virtual void cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data);
  virtual void engineerUiDataCallback(const rm_msgs::EngineerUi::ConstPtr& data);
  virtual void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data);
  virtual void radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data);
  virtual void cameraNameCallBack(const std_msgs::StringConstPtr& data);
  virtual void trackCallBack(const rm_msgs::TrackDataConstPtr& data);
  virtual void balanceStateCallback(const rm_msgs::BalanceStateConstPtr& data);
  virtual void radarReceiveCallback(const rm_msgs::ClientMapReceiveData::ConstPtr& data);
  virtual void mapSentryCallback(const rm_msgs::MapSentryDataConstPtr& data);
  virtual void sentryDeviateCallback(const rm_msgs::SentryDeviateConstPtr& data);
  virtual void sendCurrentSentryCallback(const rm_msgs::CurrentSentryPosDataConstPtr& data);
  virtual void sendSentryCmdCallback(const rm_msgs::SentryInfoConstPtr& data);
  virtual void sendRadarCmdCallback(const rm_msgs::RadarInfoConstPtr& data);
  virtual void sendSentryStateCallback(const std_msgs::StringConstPtr& data);
  virtual void dronePoseCallBack(const geometry_msgs::PoseStampedConstPtr& data);

  // send  ui
  void sendSerialDataCallback();
  void sendQueue();

  ros::Subscriber joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber chassis_cmd_sub_;
  ros::Subscriber vel2D_cmd_sub_;
  ros::Subscriber shoot_state_sub_;
  ros::Subscriber gimbal_cmd_sub_;
  ros::Subscriber detection_status_sub_;
  ros::Subscriber card_cmd_sub_;
  ros::Subscriber calibration_status_sub_;
  ros::Subscriber engineer_cmd_sub_;
  ros::Subscriber radar_date_sub_;
  ros::Subscriber manual_data_sub_;
  ros::Subscriber camera_name_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber balance_state_sub_;
  ros::Subscriber radar_receive_sub_;
  ros::Subscriber map_sentry_sub_;
  ros::Subscriber radar_to_sentry_sub_;
  ros::Subscriber sentry_cmd_sub_;
  ros::Subscriber radar_cmd_sub_;
  ros::Subscriber sentry_state_sub_;
  ros::Subscriber drone_pose_sub_;

  ChassisTriggerChangeUi* chassis_trigger_change_ui_{};
  ShooterTriggerChangeUi* shooter_trigger_change_ui_{};
  GimbalTriggerChangeUi* gimbal_trigger_change_ui_{};
  TargetTriggerChangeUi* target_trigger_change_ui_{};
  TargetViewAngleTriggerChangeUi* target_view_angle_trigger_change_ui_{};
  CameraTriggerChangeUi* camera_trigger_change_ui_{};

  BulletTimeChangeUi* bullet_time_change_ui_{};
  CapacitorTimeChangeUi* capacitor_time_change_ui_{};
  EffortTimeChangeUi* effort_time_change_ui_{};
  ProgressTimeChangeUi* progress_time_change_ui_{};
  DartStatusTimeChangeUi* dart_status_time_change_ui_{};
  RotationTimeChangeUi* rotation_time_change_ui_{};
  LaneLineTimeChangeGroupUi* lane_line_time_change_ui_{};
  BalancePitchTimeChangeGroupUi* balance_pitch_time_change_group_ui_{};
  PitchAngleTimeChangeUi* pitch_angle_time_change_ui_{};
  JointPositionTimeChangeUi *engineer_joint1_time_change_ui{}, *engineer_joint2_time_change_ui{},
      *engineer_joint3_time_change_ui{};
  TargetDistanceTimeChangeUi* target_distance_time_change_ui_{};

  DroneTowardsTimeChangeGroupUi* drone_towards_time_change_group_ui_{};
  StringTriggerChangeUi *servo_mode_trigger_change_ui_{}, *stone_num_trigger_change_ui_{},
      *joint_temperature_trigger_change_ui_{}, *gripper_state_trigger_change_ui_{};

  FixedUi* fixed_ui_{};

  CoverFlashUi* cover_flash_ui_{};
  SpinFlashUi* spin_flash_ui_{};
  HeroHitFlashUi* hero_hit_flash_ui_{};
  ExceedBulletSpeedFlashUi* exceed_bullet_speed_flash_ui_{};

  InteractiveSender* interactive_data_sender_{};
  InteractiveSender* enemy_hero_state_sender_{};
  InteractiveSender* sentry_state_sender_{};

  GroupUiBase* graph_queue_sender_{};
  std::deque<Graph> graph_queue_;
  std::deque<Graph> character_queue_;

  ros::Time radar_interactive_data_last_send_;
  ros::Time sentry_interactive_data_last_send_;
  ros::Time sentry_cmd_data_last_send_, radar_cmd_data_last_send_;

  Base& base_;
  ros::Timer add_ui_timer_, send_serial_data_timer_;
  int add_ui_times_, add_ui_max_times_, add_ui_frequency_;
  double send_ui_queue_delay_;
  bool add_ui_flag_ = false, is_adding_ = false;
  ros::NodeHandle nh_;
  std::string dbus_topic_;
};
}  // namespace rm_referee
