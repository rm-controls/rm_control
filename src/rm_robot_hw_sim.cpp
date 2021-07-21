//
// Created by qiayuan on 2/10/21.
//

#include "rm_gazebo/rm_robot_hw_sim.h"

#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace rm_gazebo {

bool RmRobotHWSim::initSim(const std::string &robot_namespace,
                           ros::NodeHandle model_nh,
                           gazebo::physics::ModelPtr parent_model,
                           const urdf::Model *urdf_model,
                           std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&robot_state_interface_);
  return ret;
}

void RmRobotHWSim::readSim(ros::Time time, ros::Duration period) {
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto &cmd:joint_effort_command_)
    cmd = 0;
  for (auto &cmd:joint_velocity_command_)
    cmd = 0;
}

}

PLUGINLIB_EXPORT_CLASS(rm_gazebo::RmRobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin) // Default plugin