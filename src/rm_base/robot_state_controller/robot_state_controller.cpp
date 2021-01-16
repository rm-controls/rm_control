//
// Created by qiayuan on 1/3/21.
//
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include "rm_base/robot_state_controller/robot_state_controller.h"

namespace robot_state_controller {

bool RobotStateController::init(hardware_interface::RobotHW *robot_hw,
                                ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  // set publish frequency
  controller_nh.param("publish_rate", publish_rate_, 50.0);
  // set whether to use the /tf_static latched static transform broadcaster
  controller_nh.param("use_tf_static", use_tf_static_, true);
  // ignore_timestamp_ == true, joins_states messages are accepted, no matter their timestamp
  controller_nh.param("ignore_timestamp", ignore_timestamp_, false);
  double duration;
  controller_nh.param("buffer_duration", duration, 1.);

  if (!model_.initParam("robot_description"))
    return false;
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  addChildren(tree.getRootSegment());

  mimic_ = new std::map<std::string, urdf::JointMimicSharedPtr>;
  for (auto &joint : model_.joints_)
    if (joint.second->mimic)
      mimic_->insert(std::make_pair(joint.first, joint.second->mimic));

  const std::vector<std::string> &joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
  num_hw_joints_ = joint_names.size();
  for (unsigned i = 0; i < num_hw_joints_; i++)
    jnt_states_.insert(std::make_pair<std::string, hardware_interface::JointStateHandle>(
        joint_names[i].c_str(), robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_names[i])));

  tf_broadcaster_.init(root_nh);
  static_tf_broadcaster_.init(root_nh);

  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(duration));
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  hardware_interface::RobotStateHandle robot_state_handle("tf_buffer", tf_buffer_);
  robot_hw->get<hardware_interface::RobotStateInterface>()->registerHandle(robot_state_handle);

  return true;
}

std::string stripSlash(const std::string &in) {
  if (!in.empty() && in[0] == '/') {
    return in.substr(1);
  }
  return in;
}

void RobotStateController::update(const ros::Time &time, const ros::Duration &) {
  std::vector<geometry_msgs::TransformStamped> tf_transforms, tf_static_transforms;
  geometry_msgs::TransformStamped tf_transform;
  // loop over all float segments
  for (auto &item:segments_) {
    auto jnt_iter = jnt_states_.find(item.first);
    auto mimic_iter = mimic_->find(item.first);
    if (jnt_iter != jnt_states_.end())
      tf_transform =
          tf2::kdlToTransform(item.second.segment.pose(jnt_iter->second.getPosition()));
    else if (mimic_iter != mimic_->end())
      tf_transform =
          tf2::kdlToTransform(item.second.segment.pose(
              jnt_states_.find(mimic_iter->first)->second.getPosition() * mimic_iter->second->multiplier
                  + mimic_iter->second->offset));
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF",
                        item.first.c_str());
      continue;
    }
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(item.second.root);
    tf_transform.child_frame_id = stripSlash(item.second.tip);
    tf_transforms.push_back(tf_transform);
  }

  // loop over all fixed segments
  for (std::map<std::string, SegmentPair>::const_iterator seg = segments_fixed_.begin(); seg != segments_fixed_.end();
       seg++) {
    tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
    tf_transform.header.stamp = time;
    tf_transform.header.frame_id = stripSlash(seg->second.root);
    tf_transform.child_frame_id = stripSlash(seg->second.tip);
    tf_static_transforms.push_back(tf_transform);
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    tf_broadcaster_.sendTransform(tf_transforms);
    if (use_tf_static_)
      static_tf_broadcaster_.sendTransform(tf_static_transforms);
    else
      tf_broadcaster_.sendTransform(tf_static_transforms);
    last_publish_time_ = time;
  }
}

// add children to correct maps
void RobotStateController::addChildren(const KDL::SegmentMap::const_iterator segment) {
  const std::string &root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator> &children = GetTreeElementChildren(segment->second);
  for (auto i : children) {
    const KDL::Segment &child = GetTreeElementSegment(i->second);
    SegmentPair s(GetTreeElementSegment(i->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName())
          && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        ROS_INFO(
            "Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info",
            root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(i);
  }
}

}

PLUGINLIB_EXPORT_CLASS(robot_state_controller::RobotStateController, controller_interface::ControllerBase)
