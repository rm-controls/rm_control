//
// Created by qiayuan on 1/2/21.
//

#ifndef RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
#define RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

#include <utility>

namespace transmission_interface {
class DoubleActuatorTransmission : public Transmission {
 public:
  DoubleActuatorTransmission(std::vector<double> actuator_reduction,
                             double joint_reduction,
                             double joint_offset = 0.0);
  void actuatorToJointEffort(const ActuatorData &act_data,
                             JointData &jnt_data) override;
  void actuatorToJointVelocity(const ActuatorData &act_data,
                               JointData &jnt_data) override;
  void actuatorToJointPosition(const ActuatorData &act_data,
                               JointData &jnt_data) override;
  void jointToActuatorEffort(const JointData &jnt_data,
                             ActuatorData &act_data) override;

  std::size_t numActuators() const override { return 2; }
  std::size_t numJoints() const override { return 1; }

  const std::vector<double> &getActuatorReduction() const { return act_reduction_; }
  double getJointReduction() const { return jnt_reduction_; }
  double getJointOffset() const { return jnt_offset_; }

 protected:
  std::vector<double> act_reduction_;
  double jnt_reduction_{};
  double jnt_offset_{};
};

DoubleActuatorTransmission::DoubleActuatorTransmission(std::vector<double> actuator_reduction,
                                                       const double joint_reduction,
                                                       const double joint_offset)
    : act_reduction_(std::move(actuator_reduction)),
      jnt_reduction_(joint_reduction),
      jnt_offset_(joint_offset) {
  if (numActuators() != act_reduction_.size() || numJoints() != 1) {
    throw TransmissionInterfaceException(
        "Joint reduction and offset vectors of a double transmission must have size 1, actuator must size 2");
  }
  if (0.0 == act_reduction_[0] || 0.0 == act_reduction_[1] || 0.0 == jnt_reduction_) {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

inline void DoubleActuatorTransmission::actuatorToJointEffort(const ActuatorData &act_data, JointData &jnt_data) {
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0]);
  const std::vector<double> &ar = act_reduction_;

  *jnt_data.effort[0] = jnt_reduction_ * (*act_data.effort[0] * ar[0] + *act_data.effort[1] * ar[1]);
}

inline void DoubleActuatorTransmission::actuatorToJointVelocity(const ActuatorData &act_data, JointData &jnt_data) {
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0]);
  const std::vector<double> &ar = act_reduction_;

  *jnt_data.velocity[0] = (*act_data.velocity[0] / ar[0] + *act_data.velocity[1] / ar[1]) / (2.0 * jnt_reduction_);
}

inline void DoubleActuatorTransmission::actuatorToJointPosition(const ActuatorData &act_data, JointData &jnt_data) {
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0]);
  const std::vector<double> &ar = act_reduction_;

  // Use first actuator's position
  *jnt_data.position[0] = (*act_data.position[0] / ar[0]) / jnt_reduction_ + jnt_offset_;
}

inline void DoubleActuatorTransmission::jointToActuatorEffort(const JointData &jnt_data, ActuatorData &act_data) {
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0]);
  const std::vector<double> &ar = act_reduction_;

  // Desired act effort is divided to two actuator evenly.
  *act_data.effort[0] = (*jnt_data.effort[0] / jnt_reduction_) / (2.0 * ar[0]);
  *act_data.effort[1] = (*jnt_data.effort[0] / jnt_reduction_) / (2.0 * ar[1]);
}

}

#endif //RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
