//
// Created by qiayuan on 12/28/20.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_
#define RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_

#include "rm_base/hardware_interface/socketcan.h"
#include "rm_base/hardware_interface/types.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace rm_base {

class CanBus {
 public:
  CanBus(const std::string &bus_name, CanDataPtr data_prt);
  void read(ros::Time time);
  void write();
 private:
  void frameCallback(const can_frame &frame);

  can::SocketCAN socket_can_;
  CanDataPtr data_prt_;
  std::string bus_name_;
  std::vector<can_frame> read_buffer_;

  can_frame rm_frame0_{};  // for id 0x201~0x204
  can_frame rm_frame1_{};  // for id 0x205~0x208

  mutable std::mutex mutex_;
};

}

#endif  //RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_
