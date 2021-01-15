//
// Created by qiayuan on 1/2/21.
//

#ifndef RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
#define RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
#include <transmission_interface/simple_transmission.h>
#include <angles/angles.h>

namespace transmission_interface {
class RevoluteTransmission : public SimpleTransmission {
 public:
  using SimpleTransmission::SimpleTransmission;
  void actuatorToJointPosition(const ActuatorData &act_data, JointData &jnt_data) override;
  void actuatorToJointAbsolutePosition(const ActuatorData &act_data, JointData &jnt_data) override;
  void jointToActuatorPosition(const JointData &jnt_data, ActuatorData &act_data) override;
};

void RevoluteTransmission::actuatorToJointPosition(const ActuatorData &act_data, JointData &jnt_data) {
  SimpleTransmission::actuatorToJointPosition(act_data, jnt_data);
  *jnt_data.position[0] = angles::normalize_angle(*jnt_data.position[0]);
}

void RevoluteTransmission::actuatorToJointAbsolutePosition(const ActuatorData &act_data, JointData &jnt_data) {
  SimpleTransmission::actuatorToJointAbsolutePosition(act_data, jnt_data);
  *jnt_data.absolute_position[0] = angles::normalize_angle(*jnt_data.absolute_position[0]);
}

void RevoluteTransmission::jointToActuatorPosition(const JointData &jnt_data, ActuatorData &act_data) {
  SimpleTransmission::jointToActuatorPosition(jnt_data, act_data);
  *act_data.position[0] =
      *act_data.position[0] - angles::normalize_angle(*act_data.position[0]) + (*act_data.position[0]);
}

}

#endif //RM_BASE_SRC_BASE_TRANSMISSION_REVOLUTE_TRANSMISSION_H_
