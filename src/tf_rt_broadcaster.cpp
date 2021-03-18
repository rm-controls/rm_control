//
// Created by qiayuan on 1/3/21.
//
#include "rm_common/tf_rt_broadcaster.h"

#include <vector>
#include <tf2_msgs/TFMessage.h>

namespace robot_state_controller {
void TfRtBroadcaster::init(ros::NodeHandle &root_nh) {
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
}

void TfRtBroadcaster::sendTransform(const geometry_msgs::TransformStamped &transform) {
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void TfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) {
  tf2_msgs::TFMessage message;
  for (const auto &transform : transforms) {
    message.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock()) {
    realtime_pub_->msg_ = message;
    realtime_pub_->unlockAndPublish();
  }
}

void StaticTfRtBroadcaster::init(ros::NodeHandle &root_nh) {
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf_static", 100, true));
}

void StaticTfRtBroadcaster::sendTransform(const geometry_msgs::TransformStamped &transform) {
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void StaticTfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) {
  for (const auto &transform : transforms) {
    bool match_found = false;
    for (auto &it_msg : net_message_.transforms) {
      if (transform.child_frame_id == it_msg.child_frame_id) {
        it_msg = transform;
        match_found = true;
        break;
      }
    }
    if (!match_found)
      net_message_.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock()) {
    realtime_pub_->msg_ = net_message_;
    realtime_pub_->unlockAndPublish();
  }
}

}// namespace robot_state_controller

