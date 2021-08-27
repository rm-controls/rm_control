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
// Created by qiayuan on 1/2/21.
//

#pragma once

#include <tinyxml.h>
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{
class DoubleActuatorTransmissionLoader : public TransmissionLoader
{
public:
  /** \brief Load transmission.
   *
   *
   *
   * @param transmission_info A pointer which point to data that stored name, type, joints, actuators of a transmission.
   * @return If successful, it will return a pointer which point to a transmission that has been loaded. If failed, it
   * is useless.
   */
  TransmissionSharedPtr load(const TransmissionInfo& transmission_info) override;

private:
  /** \brief Get actuators' configuration.
   *
   *
   *
   * @param transmission_info A pointer which point to data that stored name, type, joints, actuators of a transmission.
   * @param actuator_reduction This function will store actuator's reductiono into this variable.
   * @return If successful, return true. If failed, return false.
   */
  static bool getActuatorConfig(const TransmissionInfo& transmission_info, std::vector<double>& actuator_reduction);
  /** \brief Get joints' configuration.
   *
   *
   *
   * @param transmission_info A pointer which point to data that stored name, type, joints, actuators of a transmission.
   * @param joint_reduction This function will store joint's reduction into this variable.
   * @param joint_offset This function will store joint's offset into this variable.
   * @return Useless.
   */
  static bool getJointConfig(const TransmissionInfo& transmission_info, double& joint_reduction, double& joint_offset);
};

}  // namespace transmission_interface
