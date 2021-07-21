//
// Created by qiayuan on 5/14/21.
//

#ifndef RM_COMMON_ACTUATOR_EXTRA_INTERFACE_H_
#define RM_COMMON_ACTUATOR_EXTRA_INTERFACE_H_

#pragma once
#include <utility>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface {

class ActuatorExtraHandle {
 public:
  ActuatorExtraHandle() = default;
  ActuatorExtraHandle(std::string name, bool *halted, bool *need_calibration, bool *calibrated,
                      bool *calibration_reading, double *pos, double *offset)
      : name_(std::move(name)), halted_(halted), need_calibration_(need_calibration), calibrated_(calibrated),
        calibration_reading_(calibration_reading), pos_(pos), offset_(offset) {
    if (!halted)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. halted pointer is null.");
    if (!need_calibration)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. need_calibration  pointer is null.");
    if (!calibrated)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. calibrated pointer is null.");
    if (!calibration_reading)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. calibration reading pointer is null.");
    if (!pos)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. pos pointer is null.");
    if (!offset)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. offset pointer is null.");
  }
  std::string getName() const { return name_; }
  bool getHalted() const {
    assert(halted_);
    return *halted_;
  }
  bool getNeedCalibration() const {
    assert(need_calibration_);
    return *need_calibration_;
  }
  bool getCalibrated() const {
    assert(calibrated_);
    return *calibrated_;
  }
  bool getCalibrationReading() const {
    assert(calibration_reading_);
    return *calibration_reading_;
  }
  double getPosition() const {
    assert(pos_);
    return *pos_;
  }
  double getOffset() const {
    assert(offset_);
    return *offset_;
  }
  void setOffset(double offset) {
    *offset_ = offset;
  }
  void setCalibrated(bool calibrated) {
    *calibrated_ = calibrated;
  }
 private:
  std::string name_;
  bool *halted_ = {nullptr};
  bool *need_calibration_ = {nullptr};
  bool *calibrated_ = {nullptr};
  bool *calibration_reading_ = {nullptr};
  double *pos_{};
  double *offset_{};
};

class ActuatorExtraInterface : public HardwareResourceManager<ActuatorExtraHandle, ClaimResources> {};

}

#endif //RM_COMMON_CALIBRATION_INTERFACE_H_
