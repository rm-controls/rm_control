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

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{
class DoubleActuatorTransmission : public Transmission
{
public:
  /** \brief Check whether it has two actuators and one joint, throw error if not. Check whether Transmission reduction
   * ratios are zero, throw error if true.
   *
   * @param actuator_reduction Actuator's reduction.
   * @param joint_reduction Joint's reduction.
   * @param joint_offset Joint's offset.
   */
  DoubleActuatorTransmission(std::vector<double> actuator_reduction, double joint_reduction, double joint_offset = 0.0);

  /** \brief Set conversion from actuator to joint on effort.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from actuator to joint on velocity.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from actuator to joint on position.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data) override;
  /** \brief Set conversion from joint to actuator on effort.
   *
   * @param act_data Data of actuator.
   * @param jnt_data Data of joint.
   */
  void jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data) override;
  void jointToActuatorVelocity(const JointData& jnt_data, ActuatorData& act_data) override{};
  void jointToActuatorPosition(const JointData& jnt_data, ActuatorData& act_data) override{};

  std::size_t numActuators() const override
  {
    return 2;
  }
  std::size_t numJoints() const override
  {
    return 1;
  }

  const std::vector<double>& getActuatorReduction() const
  {
    return act_reduction_;
  }
  double getJointReduction() const
  {
    return jnt_reduction_;
  }
  double getJointOffset() const
  {
    return jnt_offset_;
  }

protected:
  std::vector<double> act_reduction_;
  double jnt_reduction_{};
  double jnt_offset_{};
};

DoubleActuatorTransmission::DoubleActuatorTransmission(std::vector<double> actuator_reduction, double joint_reduction,
                                                       double joint_offset)
  : act_reduction_(std::move(actuator_reduction)), jnt_reduction_(joint_reduction), jnt_offset_(joint_offset)
{
  if (numActuators() != act_reduction_.size() || numJoints() != 1)
  {
    throw TransmissionInterfaceException(
        "Joint reduction and offset vectors of a double transmission must have size 1, actuator must size 2");
  }
  if (0.0 == act_reduction_[0] || 0.0 == act_reduction_[1] || 0.0 == jnt_reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

inline void DoubleActuatorTransmission::actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0]);
  const std::vector<double>& ar = act_reduction_;

  *jnt_data.effort[0] = jnt_reduction_ * (*act_data.effort[0] * ar[0] + *act_data.effort[1] * ar[1]);
}

inline void DoubleActuatorTransmission::actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0]);
  const std::vector<double>& ar = act_reduction_;

  *jnt_data.velocity[0] = (*act_data.velocity[0] / ar[0] + *act_data.velocity[1] / ar[1]) / (2.0 * jnt_reduction_);
}

inline void DoubleActuatorTransmission::actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0]);
  const std::vector<double>& ar = act_reduction_;

  // Use first actuator's position
  *jnt_data.position[0] = (*act_data.position[0] / ar[0]) / jnt_reduction_ + jnt_offset_;
}

inline void DoubleActuatorTransmission::jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0]);
  const std::vector<double>& ar = act_reduction_;

  // Desired act effort is divided to two actuator evenly.
  *act_data.effort[0] = (*jnt_data.effort[0] / jnt_reduction_) / (2.0 * ar[0]);
  *act_data.effort[1] = (*jnt_data.effort[0] / jnt_reduction_) / (2.0 * ar[1]);
}

}  // namespace transmission_interface
