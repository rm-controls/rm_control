//
// Created by qiayuan on 1/3/21.
//

#ifndef RM_BASE_INCLUDE_BASE_ROBOT_STATES_CONTROLLER_ROBOT_STATE_CONTROLLER_H_
#define RM_BASE_INCLUDE_BASE_ROBOT_STATES_CONTROLLER_ROBOT_STATE_CONTROLLER_H_

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "base/robot_state_controller/tf_rt_broadcaster.h"
#include "base/hardware_interface/robot_state_interface.h"

namespace robot_state_controller {
class SegmentPair {
 public:
  SegmentPair(const KDL::Segment &p_segment, std::string p_root, std::string p_tip) :
      segment(p_segment), root(std::move(p_root)), tip(std::move(p_tip)) {}

  KDL::Segment segment{};
  std::string root, tip;
};

class RobotStateController : public controller_interface::MultiInterfaceController<
    hardware_interface::JointStateInterface, hardware_interface::RobotStateInterface> {
 public:
  RobotStateController() = default;
  bool init(hardware_interface::RobotHW *robot_hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration & /*period*/) override;
 private:
  virtual void addChildren(KDL::SegmentMap::const_iterator segment);

  urdf::Model model_{};
  std::map<std::string, urdf::JointMimicSharedPtr> *mimic_{};
  unsigned int num_hw_joints_{};
  bool use_tf_static_{};
  bool ignore_timestamp_{};
  double publish_rate_{};
  ros::Time last_publish_time_;

  std::map<std::string, hardware_interface::JointStateHandle> jnt_states_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;

  TfRtBroadcaster tf_broadcaster_;
  StaticTfRtBroadcaster static_tf_broadcaster_;

  tf2_ros::Buffer *tf_buffer_{};
  tf2_ros::TransformListener *tf_listener_{};
};

}

#endif //RM_BASE_INCLUDE_BASE_ROBOT_STATES_CONTROLLER_ROBOT_STATE_CONTROLLER_H_
