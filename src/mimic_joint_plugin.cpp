/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <rm_gazebo/mimic_joint_plugin.h>

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace gazebo {

MimicJointPlugin::MimicJointPlugin() {
  joint_.reset();
  mimic_joint_.reset();
}

MimicJointPlugin::~MimicJointPlugin() {
  update_connection_.reset();
}

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  model_ = _parent;
  world_ = model_->GetWorld();

  // Error message if the model couldn't be found
  if (!model_) {
    ROS_ERROR("Parent model is NULL! MimicJointPlugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if (!ros::isInitialized()) {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  // Check for robot namespace
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  ros::NodeHandle model_nh(robot_namespace_);

  // Check for joint element
  if (!_sdf->HasElement("joint")) {
    ROS_ERROR("No joint element present. MimicJointPlugin could not be loaded.");
    return;
  }

  joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

  // Check for mimicJoint element
  if (!_sdf->HasElement("mimicJoint")) {
    ROS_ERROR("No mimicJoint element present. MimicJointPlugin could not be loaded.");
    return;
  }

  mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  // Check for multiplier element
  multiplier_ = 1.0;
  if (_sdf->HasElement("multiplier"))
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

  // Check for offset element
  offset_ = 0.0;
  if (_sdf->HasElement("offset"))
    offset_ = _sdf->GetElement("offset")->Get<double>();

  // Check for sensitiveness element
  sensitiveness_ = 0.0;
  if (_sdf->HasElement("sensitiveness"))
    sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

  // Get pointers to joints
  joint_ = model_->GetJoint(joint_name_);
  if (!joint_) {
    ROS_ERROR_STREAM("No joint named \"" << joint_name_ << "\". MimicJointPlugin could not be loaded.");
    return;
  }
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
  if (!mimic_joint_) {
    ROS_ERROR_STREAM("No (mimic) joint named \"" << mimic_joint_name_ << "\". MimicJointPlugin could not be loaded.");
    return;
  }

  // Check for max effort
#if GAZEBO_MAJOR_VERSION > 2
  max_effort_ = mimic_joint_->GetEffortLimit(0);
#else
  max_effort_ = mimic_joint_->GetMaxForce(0);
#endif
  if (_sdf->HasElement("maxEffort")) {
    max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJointPlugin::UpdateChild, this));

  // Output some confirmation
  ROS_INFO_STREAM(
      "MimicJointPlugin loaded! Joint: \"" << joint_name_ << "\", Mimic joint: \"" << mimic_joint_name_ << "\""
                                           << ", Multiplier: " << multiplier_ << ", Offset: " << offset_
                                           << ", MaxEffort: " << max_effort_ << ", Sensitiveness: " << sensitiveness_);
}

void MimicJointPlugin::UpdateChild() {
#if GAZEBO_MAJOR_VERSION >= 8
  static ros::Duration period(world_->Physics()->GetMaxStepSize());
#else
  static ros::Duration period(world_->GetPhysicsEngine()->GetMaxStepSize());
#endif

  // Set mimic joint's angle based on joint's angle
#if GAZEBO_MAJOR_VERSION >= 8
  double angle = joint_->Position(0) * multiplier_ + offset_;
  double a = mimic_joint_->Position(0);
#else
  double angle = joint_->GetAngle(0).Radian() * multiplier_ + offset_;
  double a = mimic_joint_->GetAngle(0).Radian();
#endif

  if (fabs(angle - a) >= sensitiveness_) {
#if GAZEBO_MAJOR_VERSION >= 9
    mimic_joint_->SetPosition(0, angle, true);
#elif GAZEBO_MAJOR_VERSION > 2
    ROS_WARN_ONCE("The mimic_joint plugin is using the Joint::SetPosition method without preserving the link velocity.");
    ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
    ROS_WARN_ONCE("Please set gazebo_pid parameters or upgrade to Gazebo 9.");
    ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
    mimic_joint_->SetPosition(0, angle);
#else
    mimic_joint_->SetAngle(0, math::Angle(angle));
#endif

  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin);

}  // namespace gazebo
