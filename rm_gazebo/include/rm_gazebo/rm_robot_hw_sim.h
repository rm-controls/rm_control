//
// Created by qiayuan on 2/10/21.
//

#ifndef RM_GAZEBO_RM_ROBOT_HW_SIM_H
#define RM_GAZEBO_RM_ROBOT_HW_SIM_H
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <rm_common/hardware_interface/robot_state_interface.h>

namespace rm_gazebo {
class RmRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  bool initSim(const std::string &robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model,
               const urdf::Model *urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;

 private:
  hardware_interface::RobotStateInterface robot_state_interface_;
};
}

#endif //RM_GAZEBO_RM_ROBOT_HW_SIM_H
