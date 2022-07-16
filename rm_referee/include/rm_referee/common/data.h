// Created by ljq on 2021/12/3.
//

#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <rm_referee/referee/referee.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8MultiArray.h>

#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/CapacityData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/StatusChangeRequest.h>
#include <rm_msgs/StateCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/DetectionStatus.h>
#include <rm_msgs/CalibrationStatus.h>
#include <rm_msgs/EngineerCmd.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/DartRemainingTime.h>
#include <rm_msgs/ManualToReferee.h>

namespace rm_referee
{
class Data
{
public:
  explicit Data(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
  {
    // sub
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Data::jointStateCallback, this);
    actuator_state_sub_ =
        nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &Data::actuatorStateCallback, this);
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                         &Data::chassisCmdDataCallback, this);
    vel2D_cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Data::vel2DCmdDataCallback, this);
    shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                     &Data::shootCmdDataCallback, this);
    gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                       &Data::gimbalCmdDataCallback, this);
    detection_status_sub_ =
        nh.subscribe<rm_msgs::DetectionStatus>("/detection_status", 10, &Data::detectionStatusDataCallback, this);
    card_cmd_sub_ =
        nh.subscribe<rm_msgs::StateCmd>("/controllers/card_controller/command", 10, &Data::cardCmdDataCallback, this);
    calibration_status_sub_ =
        nh.subscribe<rm_msgs::CalibrationStatus>("/calibration_status", 10, &Data::calibrationStatusDataCallback, this);
    engineer_cmd_sub_ = nh.subscribe<rm_msgs::EngineerCmd>("/engineer_cmd", 10, &Data::engineerCmdDataCallback, this);
    manual_data_sub_ =
        nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &Data::manualDataCallBack, this);
    if (referee_.referee_data_.robot_id_ == rm_common::RobotId::RED_RADAR ||
        referee_.referee_data_.robot_id_ == rm_common::RobotId::BLUE_RADAR)
      radar_date_sub_ = nh.subscribe<std_msgs::Int8MultiArray>("/data", 10, &Data::radarDataCallBack, this);
    // pub
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    referee_.game_robot_status_pub_ = root_nh.advertise<rm_msgs::GameRobotStatus>("/game_robot_status", 1);
    referee_.game_status_pub_ = root_nh.advertise<rm_msgs::GameStatus>("/game_status", 1);
    referee_.capacity_data_pub_ = root_nh.advertise<rm_msgs::CapacityData>("/capacity_data", 1);
    referee_.power_heat_data_pub_ = root_nh.advertise<rm_msgs::PowerHeatData>("/power_heat_data", 1);
    referee_.game_robot_hp_pub_ = root_nh.advertise<rm_msgs::GameRobotHp>("/game_robot_hp", 1);
    referee_.event_data_pub_ = root_nh.advertise<rm_msgs::EventData>("/event_data", 1);
    referee_.dart_status_pub_ = root_nh.advertise<rm_msgs::DartStatus>("/dart_status_data", 1);
    referee_.icra_buff_debuff_zone_status_pub_ =
        root_nh.advertise<rm_msgs::IcraBuffDebuffZoneStatus>("/icra_buff_debuff_zone_status_data", 1);
    referee_.supply_projectile_action_pub_ =
        root_nh.advertise<rm_msgs::SupplyProjectileAction>("/supply_projectile_action_data", 1);
    referee_.dart_remaining_time_pub_ = root_nh.advertise<rm_msgs::DartRemainingTime>("/dart_remaining_time_data", 1);
    referee_.robot_hurt_pub_ = root_nh.advertise<rm_msgs::RobotHurt>("/robot_hurt_data", 1);
    referee_.shoot_data_pub_ = root_nh.advertise<rm_msgs::ShootData>("/shoot_data", 1);
    referee_.bullet_remaining_pub_ = root_nh.advertise<rm_msgs::BulletRemaining>("/bullet_remaining_data", 1);
    referee_.rfid_status_pub_ = root_nh.advertise<rm_msgs::RfidStatus>("/rfid_status_data", 1);
    referee_.dart_client_cmd_pub_ = root_nh.advertise<rm_msgs::DartClientCmd>("/dart_client_cmd_data", 1);
    // service
    initSerial();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
  {
    joint_state_ = *joint_state;
  }
  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
  {
    actuator_state_ = *data;
  }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
  {
    dbus_data_ = *data;
  }
  void chassisCmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
  {
    chassis_cmd_data_ = *data;
  }
  void vel2DCmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
  {
    vel2d_cmd_data_ = *data;
  }
  void shootCmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
  {
    shoot_cmd_data_ = *data;
  }
  void gimbalCmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
  {
    gimbal_cmd_data_ = *data;
  }
  void detectionStatusDataCallback(const rm_msgs::DetectionStatus::ConstPtr& data)
  {
    detection_status_data_ = *data;
  }
  void cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
  {
    card_cmd_data_ = *data;
  }
  void calibrationStatusDataCallback(const rm_msgs::CalibrationStatus::ConstPtr& data)
  {
    calibration_status_data_ = *data;
  }
  void engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data)
  {
    engineer_cmd_data_ = *data;
  }
  void manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
  {
    manual_to_referee_data_ = *data;
  }
  void radarDataCallBack(const std_msgs::Int8MultiArrayConstPtr& data)
  {
    radar_data_ = data->data[0] * 10 + data->data[1];
  }
  void initSerial()
  {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen())
      return;
    try
    {
      serial_.open();
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("Cannot open referee port");
    }
  }

  ros::Subscriber joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber chassis_cmd_sub_;
  ros::Subscriber vel2D_cmd_sub_;
  ros::Subscriber cover_cmd_sub_;
  ros::Subscriber shoot_cmd_sub_;
  ros::Subscriber gimbal_cmd_sub_;
  ros::Subscriber detection_status_sub_;
  ros::Subscriber card_cmd_sub_;
  ros::Subscriber calibration_status_sub_;
  ros::Subscriber engineer_cmd_sub_;
  ros::Subscriber radar_date_sub_;
  ros::Subscriber manual_data_sub_;

  uint8_t radar_data_;
  sensor_msgs::JointState joint_state_;
  rm_msgs::ActuatorState actuator_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::ChassisCmd chassis_cmd_data_;
  geometry_msgs::Twist vel2d_cmd_data_;
  rm_msgs::ShootCmd shoot_cmd_data_;
  rm_msgs::GimbalCmd gimbal_cmd_data_;
  rm_msgs::DetectionStatus detection_status_data_;
  rm_msgs::StateCmd card_cmd_data_;
  rm_msgs::CalibrationStatus calibration_status_data_;
  rm_msgs::EngineerCmd engineer_cmd_data_;
  rm_msgs::ManualToReferee manual_to_referee_data_;

  Referee referee_;
  serial::Serial serial_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rm_referee
