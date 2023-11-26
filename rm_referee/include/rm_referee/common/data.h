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

#include <ros/ros.h>
#include <unistd.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8MultiArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
#include <rm_msgs/DartRemainingTime.h>
#include <rm_msgs/StatusChangeRequest.h>
#include <rm_msgs/ClientMapReceiveData.h>
#include <rm_msgs/SupplyProjectileAction.h>
#include <rm_msgs/IcraBuffDebuffZoneStatus.h>
#include <rm_msgs/SentryDeviate.h>
#include <rm_msgs/CurrentSentryPosData.h>
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
  serial::Serial serial_;

  int client_id_ = 0;  // recipient's id
  int robot_id_ = 0;   // recent  robot's id
  int capacity_recent_mode_, capacity_expect_mode_;
  std::string robot_color_;
  bool referee_data_is_online_ = false;

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
