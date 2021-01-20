//
// Created by qiayuan on 12/28/20.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_
#define RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_

#include <string>
#include <unordered_map>

#include <socketcan_interface/socketcan.h>
#include <lp_filter.h>

#include "rm_base/hardware_interface/types.h"

namespace rm_base {

class CanBus {
 public:
  CanBus(const std::string &bus_name, CanDataPtr data_prt);
  void write();
 private:
  void frameCallback(const can::Frame &frame);
  void stateCallback(const can::State &state);

  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  can::ThreadedSocketCANInterfaceSharedPtr driver_;
  CanDataPtr data_prt_{};
  std::string bus_name_;
  can::Frame rm_frame0_;  //for id 0x201~0x204
  can::Frame rm_frame1_;  // for id 0x205~0x208
};

}

#endif  //RM_BASE_INCLUDE_RM_BASE_CAN_BUS_H_
