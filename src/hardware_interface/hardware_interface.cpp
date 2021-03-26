//
// Created by qiayuan on 12/21/20.
//

#include <rm_common/ros_utilities.h>
#include <memory>
#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "rm_base/hardware_interface/hardware_interface.h"

namespace rm_base {

bool RmBaseHardWareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  // Parse actuator coefficient specified by user (stored on ROS parameter server)
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (!robot_hw_nh.getParam("actuator_coefficient", xml_rpc_value))
    ROS_WARN("No actuator coefficient specified");
  else if (!parseActCoeffs(xml_rpc_value))
    return false;

  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("actuators", xml_rpc_value))
    ROS_WARN("No actuator specified");
  else if (!parseActData(xml_rpc_value, robot_hw_nh))
    return false;

  // Parse actuator specified by user (stored on ROS parameter server)
  if (!robot_hw_nh.getParam("imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else if (!parseImuData(xml_rpc_value, robot_hw_nh))
    return false;

  if (!load_urdf(root_nh)) {
    ROS_ERROR("Error occur when setup urdf");
    return false;
  }
  // Initialize transmission
  if (!setupTransmission(root_nh)) {
    ROS_ERROR("Error occur when setup transmission");
    return false;
  }
  // Initialize joint limit
  setupJointLimit(root_nh);
  if (!setupJointLimit(root_nh)) {
    ROS_ERROR("Error occur when setup joint limit");
    return false;
  }

  // CAN Bus
  if (!robot_hw_nh.getParam("bus", xml_rpc_value))
    ROS_WARN("No bus specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    for (int i = 0; i < xml_rpc_value.size(); ++i) {
      std::string bus_name = xml_rpc_value[i];
      if (bus_name.find("can") != std::string::npos)
        can_buses_.push_back(
            new CanBus(bus_name,
                       CanDataPtr{
                           .type2act_coeffs_=&type2act_coeffs_,
                           .id2act_data_ = &bus_id2act_data_[bus_name],
                           .id2imu_data_ = &bus_id2imu_data_[bus_name]}));
      else
        ROS_ERROR_STREAM("Unknown bus: " << bus_name);
    }
  }

  // Other Interface
  registerInterface(&robot_state_interface_);
  registerInterface(&imu_sensor_interface_);

  return true;
}

void RmBaseHardWareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // NOTE: read all data before  propagate!
  if (is_actuator_specified_)
    act_to_jnt_state_->propagate();

  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
  if (is_actuator_specified_) {
    std::vector<std::string> names = effort_joint_interface->getNames();
    for (const auto &name:names)
      effort_joint_interface->getHandle(name).setCommand(0);
  }
}

void RmBaseHardWareInterface::write(const ros::Time &time, const ros::Duration &period) {
  effort_jnt_saturation_interface_.enforceLimits(period);
  effort_jnt_soft_limits_interface_.enforceLimits(period);
  if (is_actuator_specified_)
    jnt_to_act_effort_->propagate();
  for (auto &item:can_buses_)
    item->write();
}

bool RmBaseHardWareInterface::parseActCoeffs(XmlRpc::XmlRpcValue &act_coeffs) {
  ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try {
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = act_coeffs.begin(); it != act_coeffs.end(); ++it) {
      ActCoeff act_coeff{};

      // All motor
      if (it->second.hasMember("act2pos"))
        act_coeff.act2pos = xmlRpcGetDouble(act_coeffs[it->first], "act2pos", 0.);
      else
        ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2pos.");
      if (it->second.hasMember("act2vel"))
        act_coeff.act2vel = xmlRpcGetDouble(act_coeffs[it->first], "act2vel", 0.);
      else
        ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2vel.");
      if (it->second.hasMember("act2effort"))
        act_coeff.act2effort = xmlRpcGetDouble(act_coeffs[it->first], "act2effort", 0.);
      else
        ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated act2effort.");
      if (it->second.hasMember("pos2act"))
        act_coeff.pos2act = xmlRpcGetDouble(act_coeffs[it->first], "pos2act", 0.);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated pos2act.");
      if (it->second.hasMember("vel2act"))
        act_coeff.vel2act = xmlRpcGetDouble(act_coeffs[it->first], "vel2act", 0.);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated vel2act.");
      if (it->second.hasMember("effort2act"))
        act_coeff.effort2act = xmlRpcGetDouble(act_coeffs[it->first], "effort2act", 0.0);
      else
        ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated effort2act.");
      if (it->second.hasMember("max_out"))
        act_coeff.max_out = xmlRpcGetDouble(act_coeffs[it->first], "max_out", 0.0);
      else
        ROS_ERROR_STREAM("Actuator type " << it->first << " has no associated max_out.");

      // MIT Cheetah Motor
      if (it->second.hasMember("act2pos_offset"))
        act_coeff.act2pos_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2pos_offset", -12.5);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2pos_offset.");
      if (it->second.hasMember("act2vel_offset"))
        act_coeff.act2vel_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2vel_offset", -65.0);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2vel_offset.");
      if (it->second.hasMember("act2effort_offset"))
        act_coeff.act2effort_offset = xmlRpcGetDouble(act_coeffs[it->first], "act2effort_offset", -18.0);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated act2effort_offset.");
      if (it->second.hasMember("kp2act"))
        act_coeff.kp2act = xmlRpcGetDouble(act_coeffs[it->first], "kp2act", 8.19);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated kp2act.");
      if (it->second.hasMember("kp2act"))
        act_coeff.kp2act = xmlRpcGetDouble(act_coeffs[it->first], "kd2act", 819);
      else
        ROS_DEBUG_STREAM("Actuator type " << it->first << " has no associated kd2act.");

      std::string type = it->first;
      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
        type2act_coeffs_.insert(std::make_pair(type, act_coeff));
      else
        ROS_ERROR_STREAM(
            "Repeat actuator coefficient of type: " << type);
    }
  }
  catch (XmlRpc::XmlRpcException &e) {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool RmBaseHardWareInterface::parseActData(XmlRpc::XmlRpcValue &act_datas, ros::NodeHandle &robot_hw_nh) {
  ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try {
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = act_datas.begin(); it != act_datas.end(); ++it) {
      if (!it->second.hasMember("bus")) {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated bus.");
        continue;
      } else if (!it->second.hasMember("type")) {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated type.");
        continue;
      } else if (!it->second.hasMember("id")) {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated ID.");
        continue;
      }
      std::string bus = act_datas[it->first]["bus"], type = act_datas[it->first]["type"];
      int id = static_cast<int>(act_datas[it->first]["id"]);

      // check define of act_coeffs
      if (type2act_coeffs_.find(type) == type2act_coeffs_.end()) {
        ROS_ERROR_STREAM("Type " << type << " has no associated coefficient.");
        return false;
      }

      // for bus interface
      if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
        bus_id2act_data_.insert(std::make_pair(bus, std::unordered_map<int, ActData>()));

      if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end())) {
        ROS_ERROR_STREAM("Repeat actuator on bus " << bus << " and ID " << id);
        return false;
      } else {
        ros::NodeHandle nh = ros::NodeHandle(robot_hw_nh, "actuators/" + it->first);
        bus_id2act_data_[bus].insert(
            std::make_pair(id, ActData{.type =  type, .pos = 0, .vel = 0, .effort = 0, .cmd_pos = 0,
                .cmd_vel = 0, .cmd_effort = 0, .seq = 0, .q_circle = 0, .q_last = 0, .temp = 0,
                .lp_filter=new LowPassFilter(nh)}));
      }

      // for ros_control interface
      hardware_interface::ActuatorStateHandle act_state(it->first,
                                                        &bus_id2act_data_[bus][id].pos,
                                                        &bus_id2act_data_[bus][id].vel,
                                                        &bus_id2act_data_[bus][id].effort);
      act_state_interface_.registerHandle(act_state);
      if (type.find("rm") != std::string::npos
          || type.find("cheetah") != std::string::npos) { // RoboMaster motors are effect actuator
        effort_act_interface_.registerHandle(
            hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].cmd_effort));
      } else {
        ROS_ERROR_STREAM("Actuator " << it->first <<
                                     "'s type neither RoboMaster(rm_xxx) nor Cheetah(cheetah_xxx)");
        return false;
      }

    }
    registerInterface(&act_state_interface_);
    registerInterface(&effort_act_interface_);
    is_actuator_specified_ = true;
  }
  catch (XmlRpc::XmlRpcException &e) {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool rm_base::RmBaseHardWareInterface::parseImuData(XmlRpc::XmlRpcValue &imu_datas, ros::NodeHandle &robot_hw_nh) {
  ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try {
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = imu_datas.begin(); it != imu_datas.end(); ++it) {
      if (!it->second.hasMember("frame_id")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
        continue;
      } else if (!it->second.hasMember("bus")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated bus.");
        continue;
      } else if (!it->second.hasMember("id")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated ID.");
        continue;
      } else if (!it->second.hasMember("orientation_covariance_diagonal")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
        continue;
      } else if (!it->second.hasMember("angular_velocity_covariance")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
        continue;
      } else if (!it->second.hasMember("linear_acceleration_covariance")) {
        ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
        continue;
      }
      XmlRpc::XmlRpcValue ori_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
      ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(ori_cov.size() == 3);
      for (int i = 0; i < ori_cov.size(); ++i)
        ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue angular_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
      ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(angular_cov.size() == 3);
      for (int i = 0; i < angular_cov.size(); ++i)
        ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue linear_cov = imu_datas[it->first]["linear_acceleration_covariance"];
      ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(linear_cov.size() == 3);
      for (int i = 0; i < linear_cov.size(); ++i)
        ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      std::string frame_id = imu_datas[it->first]["frame_id"],
          bus = imu_datas[it->first]["bus"];
      int id = static_cast<int>(imu_datas[it->first]["id"]);

      // for bus interface
      if (bus_id2imu_data_.find(bus) == bus_id2imu_data_.end())
        bus_id2imu_data_.insert(std::make_pair(bus, std::unordered_map<int, ImuData>()));

      if (!(bus_id2imu_data_[bus].find(id) == bus_id2imu_data_[bus].end())) {
        ROS_ERROR_STREAM("Repeat Imu on bus " << bus << " and ID " << id);
        return false;
      } else
        bus_id2imu_data_[bus].insert(
            std::make_pair(id, ImuData{
                .ori={},
                .ori_cov={
                    static_cast<double>(ori_cov[0]), 0., 0.,
                    0., static_cast<double>(ori_cov[1]), 0.,
                    0., 0., static_cast<double>(ori_cov[2])},
                .angular_vel={},
                .angular_vel_cov={
                    static_cast<double>(angular_cov[0]), 0., 0.,
                    0., static_cast<double>(angular_cov[1]), 0.,
                    0., 0., static_cast<double>(angular_cov[2])},
                .linear_acc ={},
                .linear_acc_cov={
                    static_cast<double>(linear_cov[0]), 0., 0.,
                    0., static_cast<double>(linear_cov[1]), 0.,
                    0., 0., static_cast<double>(linear_cov[2])}}));

      // for ros_control interface
      hardware_interface::ImuSensorHandle imu_sensor_handle(
          it->first, frame_id,
          bus_id2imu_data_[bus][id].ori, bus_id2imu_data_[bus][id].ori_cov,
          bus_id2imu_data_[bus][id].angular_vel, bus_id2imu_data_[bus][id].angular_vel_cov,
          bus_id2imu_data_[bus][id].linear_acc, bus_id2imu_data_[bus][id].linear_acc_cov);
      imu_sensor_interface_.registerHandle(imu_sensor_handle);

    }
    registerInterface(&imu_sensor_interface_);
  }
  catch (XmlRpc::XmlRpcException &e) {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                         << "configuration: " << e.getMessage() << ".\n"
                         << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool RmBaseHardWareInterface::load_urdf(ros::NodeHandle &root_nh) {
  if (urdf_model_ == nullptr)
    urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

bool RmBaseHardWareInterface::setupTransmission(ros::NodeHandle &root_nh) {
  if (!is_actuator_specified_) return true;
  try {
    transmission_loader_ = std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
        this, &robot_transmissions_);
  }
  catch (const std::invalid_argument &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch (const pluginlib::LibraryLoadException &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Perform transmission loading
  if (!transmission_loader_->load(urdf_string_)) { return false; }
  act_to_jnt_state_ = robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_effort_ = robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>();
  return true;
}

bool RmBaseHardWareInterface::setupJointLimit(ros::NodeHandle &root_nh) {
  if (!is_actuator_specified_) return true;
  auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
  std::vector<std::string> names = effort_joint_interface->getNames();
  joint_limits_interface::JointLimits joint_limits; // Position
  joint_limits_interface::SoftJointLimits soft_limits; // Soft Position

  for (const auto &name: names) {
    bool has_joint_limits{}, has_soft_limits{};

    hardware_interface::JointHandle joint_handle = effort_joint_interface->getHandle(name);
    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(name);
    if (urdf_joint == nullptr) {
      ROS_ERROR_STREAM("URDF joint not found " << name);
      return false;
    }
    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
      has_joint_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
    } else if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
    // Get soft limits from URDF
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
      has_soft_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
    } else
      ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from URDF.");
    // Get limits from ROS param
    if (joint_limits_interface::getJointLimits(joint_handle.getName(), root_nh, joint_limits)) {
      has_joint_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has rosparam position limits.");
    }
    // Get soft limits from ROS param
    if (joint_limits_interface::getSoftJointLimits(joint_handle.getName(), root_nh, soft_limits)) {
      has_soft_limits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from ROS param.");
    } else
      ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from ROS param.");

    // Slightly reduce the joint limits to prevent floating point errors
    if (joint_limits.has_position_limits) {
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();
    }
    if (has_soft_limits) { // Use soft limits
      ROS_DEBUG_STREAM("Using soft saturation limits");
      effort_jnt_soft_limits_interface_.registerHandle(
          joint_limits_interface::EffortJointSoftLimitsHandle(joint_handle, joint_limits, soft_limits));
    } else if (has_joint_limits) {
      ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
      effort_jnt_saturation_interface_.registerHandle(
          joint_limits_interface::EffortJointSaturationHandle(joint_handle, joint_limits));
    }
  }
  return true;
}

}