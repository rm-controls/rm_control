//
// Created by qiayuan on 12/28/20.
//
#include <string>
#include <memory>

#include <ros/ros.h>
#include <socketcan_interface/threading.h>
#include <math_utilities.h>

#include "rm_base/hardware_interface/can_bus.h"

namespace rm_base {

float int16ToFloat(unsigned short data) {
  if (data == 0)
    return 0;
  float *fp32;
  unsigned int fInt32 = ((data & 0x8000) << 16) |
      (((((data >> 10) & 0x1f) - 0x0f + 0x7f) & 0xff) << 23) | ((data & 0x03FF) << 13);
  fp32 = (float *) &fInt32;
  return *fp32;
}

CanBus::CanBus(const std::string &bus_name, CanDataPtr data_prt)
    : data_prt_(data_prt), bus_name_(bus_name) {
  driver_ = std::make_shared<can::ThreadedSocketCANInterface>();
  // Initialize device at can_device, false for no loop back.
  if (!driver_->init(bus_name, false, can::NoSettings::create()))
    ROS_FATAL("Failed to initialize can_device at %s", bus_name.c_str());
  else
    ROS_INFO("Successfully connected to %s.", bus_name.c_str());

  // Register handler for frames and state changes.
  frame_listener_ = driver_->createMsgListenerM(this, &CanBus::frameCallback);
  state_listener_ = driver_->createStateListenerM(this, &CanBus::stateCallback);
  // Set up CAN package header
  rm_frame0_.id = 0x200;
  rm_frame0_.dlc = 8;

  rm_frame1_.id = 0x1FF;
  rm_frame1_.dlc = 8;
}

void CanBus::write() {
  if (!driver_->getState().isReady())
    return;
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
    }
  }

  if (has_write_frame0)
    driver_->send(rm_frame0_);
  if (has_write_frame1)
    driver_->send(rm_frame1_);
}

void CanBus::frameCallback(const can::Frame &frame) {
  if (data_prt_.id2act_data_->find(frame.id) != data_prt_.id2act_data_->end()) {
    ActData &act_data = data_prt_.id2act_data_->find(frame.id)->second;

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

      const ActCoeff &act_coeff = data_prt_.type2act_coeffs_->find(act_data.type)->second;
      // Converter raw CAN data to position velocity effort.
      act_data.pos = act_coeff.act2pos * static_cast<double> (q + 8191 * act_data.q_circle);
      act_data.vel = act_coeff.act2vel * static_cast<double> (qd);
      act_data.effort = act_coeff.act2effort * static_cast<double> (cur);
      act_data.temp = temp;

      // Low pass filt
      act_data.lp_filter->input(act_data.vel);
      act_data.vel = act_data.lp_filter->output();
      return;
    }
  } else {
    // Check if imu
    float imu_frame_data[4] = {0};
    for (int i = 0; i < 4; ++i)
      imu_frame_data[i] = int16ToFloat((frame.data[i * 2] << 8) | frame.data[i * 2 + 1]);

    for (auto &itr :*data_prt_.id2imu_data_) { // imu data are consisted of three frames
      switch (frame.id - static_cast<unsigned int>(itr.first)) {
        case 0:itr.second.acc[0] = imu_frame_data[0];
          itr.second.acc[1] = imu_frame_data[1];
          itr.second.acc[2] = imu_frame_data[2];
          itr.second.gyr[0] = imu_frame_data[3];
          return;
        case 1:itr.second.gyr[1] = imu_frame_data[0];
          itr.second.gyr[2] = imu_frame_data[1];
          itr.second.quat[0] = imu_frame_data[2];
          itr.second.quat[1] = imu_frame_data[3];
          return;
        case 2:itr.second.quat[2] = imu_frame_data[0];
          itr.second.quat[3] = imu_frame_data[1];
          return;
        default:break;
      }
    }

    ROS_WARN_STREAM_ONCE("Can not find defined device, id: 0x" << std::hex << frame.id << " on bus: " << bus_name_);
  }
}

void CanBus::stateCallback(const can::State &state) {
  std::string err;
  driver_->translateError(state.internal_error, err);
  if (!state.internal_error) {
    ROS_INFO("State: %s, asio: %s, %s", err.c_str(), state.error_code.message().c_str(), bus_name_.c_str());
  } else {
    ROS_ERROR("Error: %s, asio: %s, %s", err.c_str(), state.error_code.message().c_str(), bus_name_.c_str());
  }
}

}
