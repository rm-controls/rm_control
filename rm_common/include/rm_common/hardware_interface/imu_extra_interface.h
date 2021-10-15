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
// Created by qiayuan on 15/9/21.
//

#pragma once

#include <utility>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rm_control
{
class ImuExtraHandle
{
public:
  ImuExtraHandle() = default;
  ImuExtraHandle(std::string name, double* orientation, bool* accel_update, bool* gyro_update, bool* camera_trigger,
                 const double* temperature)
    : name_(std::move(name))
    , orientation_(orientation)
    , accel_updated_(accel_update)
    , gyro_updated_(gyro_update)
    , camera_trigger_(camera_trigger)
    , temperature_(temperature)
  {
    if (!accel_update)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. accel_update pointer is null.");
    if (!gyro_update)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. gyro_update pointer is null.");
    if (!camera_trigger)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. camera_trigger pointer is null.");
    if (!temperature)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. camera_trigger pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  void setOrientation(double ori_x, double ori_y, double ori_z, double ori_w)
  {
    orientation_[0] = ori_x;
    orientation_[1] = ori_y;
    orientation_[2] = ori_z;
    orientation_[3] = ori_w;
  }
  bool getAccelUpdated() const
  {
    assert(accel_updated_);
    return *accel_updated_;
  }
  bool getGyroUpdated() const
  {
    assert(gyro_updated_);
    return *gyro_updated_;
  }
  bool getCameraTrigger() const
  {
    assert(camera_trigger_);
    return *camera_trigger_;
  }
  double getTemperature() const
  {
    assert(temperature_);
    return *temperature_;
  }

private:
  std::string name_;
  double* orientation_;
  bool* accel_updated_;
  bool* gyro_updated_;
  bool* camera_trigger_;
  const double* temperature_{};
  // TODO: Add magnetic (double* )
};

class ImuExtraInterface
  : public hardware_interface::HardwareResourceManager<ImuExtraHandle, hardware_interface::ClaimResources>
{
};

}  // namespace rm_control
