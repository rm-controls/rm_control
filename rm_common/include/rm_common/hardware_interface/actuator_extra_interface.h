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
// Created by qiayuan on 5/14/21.
//

#pragma once

#include <utility>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class ActuatorExtraHandle
{
public:
  ActuatorExtraHandle() = default;
  ActuatorExtraHandle(std::string name, bool* halted, bool* need_calibration, bool* calibrated,
                      bool* calibration_reading, double* pos, double* offset)
    : name_(std::move(name))
    , halted_(halted)
    , need_calibration_(need_calibration)
    , calibrated_(calibrated)
    , calibration_reading_(calibration_reading)
    , pos_(pos)
    , offset_(offset)
  {
    if (!halted)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. halted pointer is null.");
    if (!need_calibration)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. need_calibration  pointer is null.");
    if (!calibrated)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. calibrated pointer is null.");
    if (!calibration_reading)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. calibration reading pointer is null.");
    if (!pos)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. pos pointer is null.");
    if (!offset)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. offset pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  bool getHalted() const
  {
    assert(halted_);
    return *halted_;
  }
  bool getNeedCalibration() const
  {
    assert(need_calibration_);
    return *need_calibration_;
  }
  bool getCalibrated() const
  {
    assert(calibrated_);
    return *calibrated_;
  }
  bool getCalibrationReading() const
  {
    assert(calibration_reading_);
    return *calibration_reading_;
  }
  double getPosition() const
  {
    assert(pos_);
    return *pos_;
  }
  double getOffset() const
  {
    assert(offset_);
    return *offset_;
  }
  void setOffset(double offset)
  {
    *offset_ = offset;
  }
  void setCalibrated(bool calibrated)
  {
    *calibrated_ = calibrated;
  }

private:
  std::string name_;
  bool* halted_ = { nullptr };
  bool* need_calibration_ = { nullptr };
  bool* calibrated_ = { nullptr };
  bool* calibration_reading_ = { nullptr };
  double* pos_{};
  double* offset_{};
};

class ActuatorExtraInterface
  : public hardware_interface::HardwareResourceManager<ActuatorExtraHandle, hardware_interface::ClaimResources>
{
};

}  // namespace rm_control
