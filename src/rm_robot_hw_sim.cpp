//
// Created by qiayuan on 2/10/21.
//

#include "rm_gazebo/rm_robot_hw_sim.h"
namespace rm_gazebo {

bool RmRobotHWSim::initSim(const std::string &robot_namespace,
                           ros::NodeHandle model_nh,
                           gazebo::physics::ModelPtr parent_model,
                           const urdf::Model *const urdf_model,
                           std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&robot_state_interface_);
  return ret;
}

}

PLUGINLIB_EXPORT_CLASS(rm_gazebo::RmRobotHWSim, gazebo_ros_control::RobotHWSim)