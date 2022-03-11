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
// Created by qiayuan on 12/28/20.
//
#include "rm_hw/hardware_interface/can_bus.h"

#include <string>
#include <ros/ros.h>
#include <rm_common/math_utilities.h>
namespace rm_hw
{
CanBus::CanBus(const std::string& bus_name, CanDataPtr data_ptr) : data_ptr_(data_ptr), bus_name_(bus_name)
{
  // Initialize device at can_device, false for no loop back.
  while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1)) && ros::ok())
    ros::Duration(.5).sleep();

  ROS_INFO("Successfully connected to %s.", bus_name.c_str());
  // Set up CAN package header
  rm_frame0_.can_id = 0x200;
  rm_frame0_.can_dlc = 8;
  rm_frame1_.can_id = 0x1FF;
  rm_frame1_.can_dlc = 8;
}

void CanBus::write()
{
  bool has_write_frame0 = false, has_write_frame1 = false;
  // safety first
  std::fill(std::begin(rm_frame0_.data), std::end(rm_frame0_.data), 0);
  std::fill(std::begin(rm_frame1_.data), std::end(rm_frame1_.data), 0);

  for (auto& item : *data_ptr_.id2act_data_)
  {
    if (item.second.type.find("rm") != std::string::npos)
    {
      if (item.second.halted)
        continue;
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      int id = item.first - 0x201;
      double cmd =
          minAbs(act_coeff.effort2act * item.second.exe_effort, act_coeff.max_out);  // add max_range to act_data
      if (-1 < id && id < 4)
      {
        rm_frame0_.data[2 * id] = (uint8_t)(static_cast<int16_t>(cmd) >> 8u);
        rm_frame0_.data[2 * id + 1] = (uint8_t)cmd;
        has_write_frame0 = true;
      }
      else if (3 < id && id < 8)
      {
        rm_frame1_.data[2 * (id - 4)] = (uint8_t)(static_cast<int16_t>(cmd) >> 8u);
        rm_frame1_.data[2 * (id - 4) + 1] = (uint8_t)cmd;
        has_write_frame1 = true;
      }
    }
    else if (item.second.type.find("cheetah") != std::string::npos)
    {
      can_frame frame{};
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      frame.can_id = item.first;
      frame.can_dlc = 8;
      uint16_t q_des = (int)(act_coeff.pos2act * (item.second.cmd_pos - act_coeff.act2pos_offset));
      uint16_t qd_des = (int)(act_coeff.vel2act * (item.second.cmd_vel - act_coeff.act2vel_offset));
      uint16_t kp = 0.;
      uint16_t kd = 0.;
      uint16_t tau = (int)(act_coeff.effort2act * (item.second.exe_effort - act_coeff.act2effort_offset));
      // TODO(qiayuan) add position vel and effort hardware interface for MIT Cheetah Motor, now we using it as an effort joint.
      frame.data[0] = q_des >> 8;
      frame.data[1] = q_des & 0xFF;
      frame.data[2] = qd_des >> 4;
      frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
      frame.data[4] = kp & 0xFF;
      frame.data[5] = kd >> 4;
      frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
      frame.data[7] = tau & 0xff;
      socket_can_.write(&frame);
    }
  }

  if (has_write_frame0)
    socket_can_.write(&rm_frame0_);
  if (has_write_frame1)
    socket_can_.write(&rm_frame1_);
}

void CanBus::read(ros::Time time)
{
  std::lock_guard<std::mutex> guard(mutex_);

  for (auto& imu : *data_ptr_.id2imu_data_)
  {
    imu.second.gyro_updated = false;
    imu.second.accel_updated = false;
  }

  for (const auto& frame_stamp : read_buffer_)
  {
    can_frame frame = frame_stamp.frame;
    // Check if robomaster motor
    if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end())
    {
      ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
      if (act_data.type.find("rm") != std::string::npos)
      {
        act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
        act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
        int16_t cur = (frame.data[4] << 8u) | frame.data[5];
        act_data.temp = frame.data[6];

        // Multiple circle
        if (act_data.seq != 0)  // not the first receive
        {
          if (act_data.q_raw - act_data.q_last > 4096)
            act_data.q_circle--;
          else if (act_data.q_raw - act_data.q_last < -4096)
            act_data.q_circle++;
        }
        try
        {  // Duration will be out of dual 32-bit range while motor failure
          act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
        }
        catch (std::runtime_error& ex)
        {
        }
        act_data.stamp = frame_stamp.stamp;
        act_data.seq++;
        act_data.q_last = act_data.q_raw;
        // Converter raw CAN data to position velocity and effort.
        act_data.pos =
            act_coeff.act2pos * static_cast<double>(act_data.q_raw + 8191 * act_data.q_circle) + act_data.offset;
        act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
        act_data.effort = act_coeff.act2effort * static_cast<double>(cur);
        // Low pass filter
        act_data.lp_filter->input(act_data.vel);
        act_data.vel = act_data.lp_filter->output();
        continue;
      }
    }
    // Check MIT Cheetah motor
    else if (frame.can_id == static_cast<unsigned int>(0x000))
    {
      if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
      {
        ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
        const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
        if (act_data.type.find("cheetah") != std::string::npos)
        {  // MIT Cheetah Motor
          act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
          uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
          uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
          // Multiple cycle
          // NOTE: The raw data range is -4pi~4pi
          if (act_data.seq != 0)  // not the first receive
          {
            double pos_new = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                             static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
            if (pos_new - act_data.pos > 4 * M_PI)
              act_data.q_circle--;
            else if (pos_new - act_data.pos < -4 * M_PI)
              act_data.q_circle++;
          }
          try
          {  // Duration will be out of dual 32-bit range while motor failure
            act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
          }
          catch (std::runtime_error& ex)
          {
          }
          act_data.stamp = frame_stamp.stamp;
          act_data.seq++;
          act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                         static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
          // Converter raw CAN data to position velocity and effort.
          act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
          act_data.effort = act_coeff.act2effort * static_cast<double>(cur) + act_coeff.act2effort_offset;
          // Low pass filter
          act_data.lp_filter->input(act_data.vel);
          act_data.vel = act_data.lp_filter->output();
          continue;
        }
      }
    }
    else if (data_ptr_.id2imu_data_->find(frame.can_id) != data_ptr_.id2imu_data_->end())  // Check if IMU gyro
    {
      ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id)->second;
      imu_data.gyro_updated = true;
      imu_data.angular_vel[0] = ((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.angular_vel_coeff;
      imu_data.angular_vel[1] = ((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.angular_vel_coeff;
      imu_data.angular_vel[2] = ((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.angular_vel_coeff;
      int16_t temp = (int16_t)((frame.data[6] << 3) | (frame.data[7] >> 5));
      if (temp > 1023)
        temp -= 2048;
      imu_data.temperature = temp * imu_data.temp_coeff + imu_data.temp_offset;
      continue;
    }
    else if (data_ptr_.id2imu_data_->find(frame.can_id - 1) != data_ptr_.id2imu_data_->end())  // Check if IMU accel
    {
      ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id - 1)->second;
      imu_data.accel_updated = true;
      imu_data.linear_acc[0] = ((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.accel_coeff;
      imu_data.linear_acc[1] = ((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.accel_coeff;
      imu_data.linear_acc[2] = ((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.accel_coeff;
      imu_data.camera_trigger = frame.data[6];
      continue;
    }
    else if (data_ptr_.id2tof_data_->find(frame.can_id) != data_ptr_.id2tof_data_->end())
    {
      TofData& tof_data = data_ptr_.id2tof_data_->find(frame.can_id)->second;
      tof_data.distance = (((frame.data[2]) << 16) | (frame.data[1] << 8) | frame.data[0]) / 1000.;
      tof_data.dis_status = frame.data[3];
      tof_data.signal_strength = ((int16_t)((frame.data[5]) << 8) | frame.data[4]);
      continue;
    }
    if (frame.can_id != 0x0)
      ROS_ERROR_STREAM_ONCE("Can not find defined device, id: 0x" << std::hex << frame.can_id
                                                                  << " on bus: " << bus_name_);
  }
  read_buffer_.clear();
}

void CanBus::frameCallback(const can_frame& frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = ros::Time::now() };
  read_buffer_.push_back(can_frame_stamp);
}

}  // namespace rm_hw
