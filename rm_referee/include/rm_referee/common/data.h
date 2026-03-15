/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by peter on 2021/7/17.
//

#pragma once

#include <array>
#include <vector>
#include <ros/ros.h>
#include <unistd.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8MultiArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/UInt32.h"
#include "rm_msgs/VisualizeStateData.h"

#include "rm_referee/common/protocol.h"

#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/ShootState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/StateCmd.h>
#include <rm_msgs/EventData.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/RobotHurt.h>
#include <rm_msgs/ShootData.h>
#include <rm_msgs/DartStatus.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/RfidStatus.h>
#include <rm_msgs/EngineerUi.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/BalanceState.h>
#include <rm_msgs/DartClientCmd.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/MapSentryData.h>
#include <rm_msgs/RadarMarkData.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/BulletAllowance.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/ManualToReferee.h>
#include <rm_msgs/ClientMapSendData.h>
#include <rm_msgs/RobotsPositionData.h>
#include <rm_msgs/DartInfo.h>
#include <rm_msgs/StatusChangeRequest.h>
#include <rm_msgs/ClientMapReceiveData.h>
#include <rm_msgs/SupplyProjectileAction.h>
#include <rm_msgs/IcraBuffDebuffZoneStatus.h>
#include <rm_msgs/GameRobotPosData.h>
#include "rm_msgs/SentryInfo.h"
#include "rm_msgs/SentryCmd.h"
#include "rm_msgs/RadarInfo.h"
#include "rm_msgs/RadarCmd.h"
#include "rm_msgs/Buff.h"
#include "rm_msgs/TrackData.h"
#include "rm_msgs/SentryAttackingTarget.h"
#include "rm_msgs/RadarToSentry.h"
#include <rm_msgs/PowerManagementSampleAndStatusData.h>
#include <rm_msgs/PowerManagementSystemExceptionData.h>
#include <rm_msgs/PowerManagementInitializationExceptionData.h>
#include <rm_msgs/PowerManagementProcessStackOverflowData.h>
#include <rm_msgs/PowerManagementUnknownExceptionData.h>

namespace rm_referee
{
struct CapacityData
{
  double chassis_power;
  double limit_power;
  double buffer_power;
  double cap_power;
  bool is_online = false;
};

class Base
{
public:
  static constexpr size_t kMaxSerialPorts = 2;

  struct SerialPortState
  {
    serial::Serial serial;
    std::string device;
    ros::Time last_valid_frame_time;
    ros::Time next_open_attempt_time;
    bool configured = false;
  };

  std::array<SerialPortState, kMaxSerialPorts> serial_ports_{};
  size_t serial_port_count_ = 0;
  int active_port_index_ = -1;
  ros::Duration reopen_interval_{ 1.0 };

  int client_id_ = 0;  // recipient's id
  int robot_id_ = 0;   // recent  robot's id
  int capacity_recent_mode_, capacity_expect_mode_;
  std::string robot_color_;
  bool referee_data_is_online_ = false;

  void initSerial(ros::NodeHandle& nh)
  {
    std::vector<std::string> devices;
    if (!nh.getParam("serial_ports", devices) || devices.empty())
      devices = { "/dev/usbReferee" };
    if (devices.size() > kMaxSerialPorts)
    {
      ROS_WARN("rm_referee serial_ports size %zu exceeds max %zu, truncating", devices.size(), kMaxSerialPorts);
      devices.resize(kMaxSerialPorts);
    }
    serial_port_count_ = 0;
    for (const auto& device : devices)
    {
      if (device.empty())
        continue;
      auto& port = serial_ports_[serial_port_count_++];
      port.device = device;
      port.configured = true;
      port.last_valid_frame_time = ros::Time();
      port.next_open_attempt_time = ros::Time();
    }
    if (serial_port_count_ == 0)
    {
      auto& port = serial_ports_[0];
      port.device = "/dev/usbReferee";
      port.configured = true;
      serial_port_count_ = 1;
    }
    for (size_t i = 0; i < serial_port_count_; ++i)
      ensurePortOpen(i, ros::Time::now());
  }

  bool ensurePortOpen(size_t index, const ros::Time& now)
  {
    if (index >= serial_port_count_)
      return false;
    auto& port = serial_ports_[index];
    if (!port.configured)
      return false;
    if (port.serial.isOpen())
      return true;
    if (!port.next_open_attempt_time.isZero() && now < port.next_open_attempt_time)
      return false;

    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    port.serial.setPort(port.device);
    port.serial.setBaudrate(115200);
    port.serial.setTimeout(timeout);
    try
    {
      port.serial.open();
      port.next_open_attempt_time = ros::Time();
      ROS_INFO_STREAM("Opened referee port: " << port.device);
      return true;
    }
    catch (const std::exception& e)
    {
      port.next_open_attempt_time = now + reopen_interval_;
      ROS_WARN_STREAM("Cannot open referee port " << port.device << ": " << e.what());
      return false;
    }
  }

  void handlePortIoFailure(size_t index, const ros::Time& now, const std::string& action, const std::exception& e)
  {
    if (index >= serial_port_count_)
      return;
    auto& port = serial_ports_[index];
    if (port.serial.isOpen())
      port.serial.close();
    port.last_valid_frame_time = ros::Time();
    if (active_port_index_ == static_cast<int>(index))
      active_port_index_ = -1;
    port.next_open_attempt_time = now + reopen_interval_;
    ROS_WARN_STREAM("Referee port " << port.device << " " << action << " failed: " << e.what());
  }

  bool isPortFresh(size_t index, const ros::Time& now, const ros::Duration& freshness) const
  {
    if (index >= serial_port_count_)
      return false;
    const auto& port = serial_ports_[index];
    return !port.last_valid_frame_time.isZero() && (now - port.last_valid_frame_time) <= freshness;
  }

  bool isActivePortFresh(const ros::Time& now, const ros::Duration& freshness) const
  {
    return active_port_index_ >= 0 && isPortFresh(static_cast<size_t>(active_port_index_), now, freshness);
  }

  bool shouldProcessPort(size_t index, const ros::Time& now, const ros::Duration& freshness) const
  {
    if (index >= serial_port_count_)
      return false;
    if (active_port_index_ < 0)
      return true;
    return static_cast<size_t>(active_port_index_) == index || !isActivePortFresh(now, freshness);
  }

  void activatePort(size_t index)
  {
    if (index >= serial_port_count_)
      return;
    if (active_port_index_ == static_cast<int>(index))
      return;
    const std::string from =
        active_port_index_ >= 0 ? serial_ports_[static_cast<size_t>(active_port_index_)].device : std::string("<none>");
    active_port_index_ = static_cast<int>(index);
    ROS_INFO_STREAM("Switched active referee port from " << from << " to " << serial_ports_[index].device);
  }

  void markValidFrame(size_t index, const ros::Time& stamp, const ros::Duration& freshness)
  {
    if (index >= serial_port_count_)
      return;
    serial_ports_[index].last_valid_frame_time = stamp;
    if (active_port_index_ < 0 || !isActivePortFresh(stamp, freshness) || active_port_index_ == static_cast<int>(index))
      activatePort(index);
    referee_data_is_online_ = true;
  }

  void refreshOnlineState(const ros::Time& now, const ros::Duration& freshness)
  {
    referee_data_is_online_ = false;
    for (size_t i = 0; i < serial_port_count_; ++i)
    {
      if (isPortFresh(i, now, freshness))
      {
        referee_data_is_online_ = true;
        break;
      }
    }
  }

  bool writeActive(const uint8_t* data, size_t data_len)
  {
    if (active_port_index_ < 0)
      return false;
    const size_t active_index = static_cast<size_t>(active_port_index_);
    if (active_index >= serial_port_count_)
      return false;
    auto& port = serial_ports_[active_index];
    if (!port.serial.isOpen())
      return false;
    port.serial.write(data, data_len);
    return true;
  }

  // CRC check
  uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8)
  {
    unsigned char uc_index;
    while (dw_length--)
    {
      uc_index = uc_crc_8 ^ (*pch_message++);
      uc_crc_8 = rm_referee::kCrc8Table[uc_index];
    }
    return (uc_crc_8);
  }

  uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, rm_referee::kCrc8Init);
    return (uc_expected == pch_message[dw_length - 1]);
  }

  void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_crc;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    uc_crc = getCRC8CheckSum((unsigned char*)pch_message, dw_length - 1, rm_referee::kCrc8Init);
    pch_message[dw_length - 1] = uc_crc;
  }

  uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc)
  {
    uint8_t chData;
    if (pch_message == nullptr)
      return 0xFFFF;
    while (dw_length--)
    {
      chData = *pch_message++;
      (w_crc) = (static_cast<uint16_t>(w_crc) >> 8) ^
                rm_referee::wCRC_table[(static_cast<uint16_t>(w_crc) ^ static_cast<uint16_t>(chData)) & 0x00ff];
    }
    return w_crc;
  }

  uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t w_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    w_expected = getCRC16CheckSum(pch_message, dw_length - 2, rm_referee::kCrc16Init);
    return ((w_expected & 0xff) == pch_message[dw_length - 2] &&
            ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
  }

  void appendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t wCRC;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    wCRC = getCRC16CheckSum(static_cast<uint8_t*>(pch_message), dw_length - 2, rm_referee::kCrc16Init);
    pch_message[dw_length - 2] = static_cast<uint8_t>((wCRC & 0x00ff));
    pch_message[dw_length - 1] = static_cast<uint8_t>(((wCRC >> 8) & 0x00ff));
  }
};
}  // namespace rm_referee
