//
// Created by qiayuan on 12/28/20.
//
#include "rm_base/hardware_interface/can_bus.h"

#include <string>
#include <ros/ros.h>
#include <math_utilities.h>

rm_base::CanBus::CanBus(const std::string &bus_name, CanActDataPtr data_prt)
    : data_prt_(data_prt), bus_name_(bus_name) {
  // Initialize device at can_device, false for no loop back.
  while (!socket_can_.open(bus_name, [this](auto &&PH1) { frameCallback(std::forward<decltype(PH1)>(PH1)); })
      && ros::ok())
    ros::Duration(.5).sleep();

  ROS_INFO("Successfully connected to %s.", bus_name.c_str());
  // Set up CAN package header
  rm_frame0_.can_id = 0x200;
  rm_frame0_.can_dlc = 8;
  rm_frame1_.can_id = 0x1FF;
  rm_frame1_.can_dlc = 8;
}

void rm_base::CanBus::write() {
  bool has_write_frame0{}, has_write_frame1{};
  // safety first
  std::fill(std::begin(rm_frame0_.data), std::end(rm_frame0_.data), 0);
  std::fill(std::begin(rm_frame1_.data), std::end(rm_frame1_.data), 0);

  for (auto &item:*data_prt_.id2act_data_) {
    if (item.second.type.find("rm") != std::string::npos) {
      if (item.second.temp > 100) // check temperature
        continue;
      const ActCoeff &act_coeff = data_prt_.type2act_coeffs_->find(item.second.type)->second;
      int id = item.first - 0x201;
      double cmd = minAbs(act_coeff.effort2act * item.second.cmd_effort, act_coeff.max_out); //add max_range to act_data
      if (-1 < id && id < 4) {
        rm_frame0_.data[2 * id] = (uint8_t) (static_cast<int16_t>(cmd) >> 8u);
        rm_frame0_.data[2 * id + 1] = (uint8_t) cmd;
        has_write_frame0 = true;
      } else if (3 < id && id < 8) {
        rm_frame1_.data[2 * (id - 4)] = (uint8_t) (static_cast<int16_t>(cmd) >> 8u);
        rm_frame1_.data[2 * (id - 4) + 1] = (uint8_t) cmd;
        has_write_frame1 = true;
      }
    } else if (item.second.type.find("cheetah") != std::string::npos) {
      can_frame frame{};
      const ActCoeff &act_coeff = data_prt_.type2act_coeffs_->find(item.second.type)->second;
      frame.can_id = item.first;
      frame.can_dlc = 8;
      uint16_t q_des = (int) (act_coeff.pos2act * (item.second.cmd_pos - act_coeff.act2pos_offset));
      uint16_t qd_des = (int) (act_coeff.vel2act * (item.second.cmd_vel - act_coeff.act2vel_offset));
      uint16_t kp = 0.;
      uint16_t kd = 0.;
      uint16_t tau = (int) (act_coeff.effort2act * (item.second.cmd_effort - act_coeff.act2effort_offset));
      // TODO(qiayuan) add posistion vel and effort hardware interface for MIT Cheetah Motor.
      frame.data[0] = q_des >> 8;
      frame.data[1] = q_des & 0xFF;
      frame.data[2] = qd_des >> 4;
      frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
      frame.data[4] = kp & 0xFF;
      frame.data[5] = kd >> 4;
      frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
      frame.data[7] = tau & 0xff;
      socket_can_.wirte(&frame);
    }
  }

  if (has_write_frame0)
    socket_can_.wirte(&rm_frame0_);
  if (has_write_frame1)
    socket_can_.wirte(&rm_frame1_);
}

void rm_base::CanBus::frameCallback(const can_frame &frame) {
  if (data_prt_.id2act_data_->find(frame.can_id) != data_prt_.id2act_data_->end()) {
    ActData &act_data = data_prt_.id2act_data_->find(frame.can_id)->second;
    const ActCoeff &act_coeff = data_prt_.type2act_coeffs_->find(act_data.type)->second;

    if (act_data.type.find("rm") != std::string::npos) {      // unpack RoboMaster Motor
      uint16_t q = (frame.data[0] << 8u) | frame.data[1];
      int16_t qd = (frame.data[2] << 8u) | frame.data[3];
      int16_t cur = (frame.data[4] << 8u) | frame.data[5];
      uint8_t temp = frame.data[6];
      if (q - act_data.q_last > 4096)
        act_data.q_circle--;
      else if (q - act_data.q_last < -4096)
        act_data.q_circle++;
      act_data.q_last = q;
      // Converter raw CAN data to position velocity and effort.
      act_data.pos = act_coeff.act2pos * static_cast<double> (q + 8191 * act_data.q_circle);
      act_data.vel = act_coeff.act2vel * static_cast<double> (qd);
      act_data.effort = act_coeff.act2effort * static_cast<double> (cur);
      act_data.temp = temp;
      // Low pass filt
      act_data.lp_filter->input(act_data.vel);
      act_data.vel = act_data.lp_filter->output();
    }
  } else if (frame.can_id == static_cast<unsigned int>(0x000)) {
    if (data_prt_.id2act_data_->find(frame.data[0]) != data_prt_.id2act_data_->end()) {
      ActData &act_data = data_prt_.id2act_data_->find(frame.data[0])->second;
      const ActCoeff &act_coeff = data_prt_.type2act_coeffs_->find(act_data.type)->second;

      if (act_data.type.find("cheetah") != std::string::npos) { // MIT Cheetah Motor
        uint16_t q = (frame.data[1] << 8) | frame.data[2];
        uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
        // Converter raw CAN data to position velocity and effort.
        act_data.vel = act_coeff.act2vel * static_cast<double> (qd) + act_coeff.act2vel_offset;
        act_data.effort = act_coeff.act2effort * static_cast<double> (cur) + act_coeff.act2effort_offset;
        // Multiple cycle encoder
        // NOTE: Raw data range is -4pi~4pi
        double pos_new =
            act_coeff.act2pos * static_cast<double> (q) + act_coeff.act2pos_offset
                + static_cast<double>(act_data.q_circle) * 8 * M_PI;
        if (pos_new - act_data.pos > 4 * M_PI)
          act_data.q_circle--;
        else if (pos_new - act_data.pos < -4 * M_PI)
          act_data.q_circle++;
        act_data.pos = act_coeff.act2pos * static_cast<double> (q) + act_coeff.act2pos_offset
            + static_cast<double>(act_data.q_circle) * 8 * M_PI;

        // Low pass filt
        act_data.lp_filter->input(act_data.vel);
        act_data.vel = act_data.lp_filter->output();
      }
    }
  } else
    ROS_ERROR_STREAM_ONCE(
        "Can not find defined actuator, id: 0x" << std::hex << frame.can_id << " on bus: " << bus_name_);
}
