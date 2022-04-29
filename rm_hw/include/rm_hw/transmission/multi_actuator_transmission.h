//
// Created by luotinkai on 2021/12/5.
//

#pragma once

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{
class MultiActuatorTransmission : public Transmission
{
public:
  MultiActuatorTransmission(const TransmissionInfo& transmission_info, std::vector<double> actuator_reduction,
                            double joint_reduction, double joint_offset = 0.0);

  void actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data) override;
  void actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data) override;
  void actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data) override;
  void jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data) override;
  void jointToActuatorVelocity(const JointData& jnt_data, ActuatorData& act_data) override{};
  void jointToActuatorPosition(const JointData& jnt_data, ActuatorData& act_data) override{};

  std::size_t numActuators() const override
  {
    return act_number_;
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
  int act_number_ = 0;
};

MultiActuatorTransmission::MultiActuatorTransmission(const TransmissionInfo& transmission_info,
                                                     std::vector<double> actuator_reduction, double joint_reduction,
                                                     double joint_offset)
  : act_reduction_(std::move(actuator_reduction)), jnt_reduction_(joint_reduction), jnt_offset_(joint_offset)
{
  act_number_ = transmission_info.actuators_.size();
  for (int i = 1; i < act_number_; ++i)
  {
    if (0.0 == act_reduction_[i])
      throw TransmissionInterfaceException("Transmission actuators reduction ratios cannot be zero.");
  }
  if (0.0 == jnt_reduction_)
    throw TransmissionInterfaceException("Transmission joint reduction ratios cannot be zero.");
}
inline void MultiActuatorTransmission::actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data)
{
  const std::vector<double>& ar = act_reduction_;
  double ad = (*act_data.effort[0] * ar[0]);
  for (int i = 1; i < act_number_; ++i)
  {
    ad = ad + (*act_data.effort[i] * ar[i]);
  }
  *jnt_data.effort[0] = jnt_reduction_ * ad;
}

inline void MultiActuatorTransmission::actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data)
{
  const std::vector<double>& ar = act_reduction_;

  double ad = (*act_data.velocity[0] / ar[0]);
  for (int i = 1; i < act_number_; ++i)
  {
    ad = ad + (*act_data.velocity[i] / ar[i]);
  }
  *jnt_data.velocity[0] = ad / (2.0 * jnt_reduction_);
}

inline void MultiActuatorTransmission::actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data)
{
  const std::vector<double>& ar = act_reduction_;

  *jnt_data.position[0] = (*act_data.position[0] / ar[0]) / jnt_reduction_ + jnt_offset_;
}

inline void MultiActuatorTransmission::jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data)
{
  const std::vector<double>& ar = act_reduction_;

  for (int i = 0; i < act_number_; ++i)
  {
    *act_data.effort[i] = (*jnt_data.effort[0] / jnt_reduction_) / (act_number_ * ar[i]);
  }
}

}  // namespace transmission_interface
