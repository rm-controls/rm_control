//
// Created by qiayuan on 1/3/21.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_ROBOT_STATE_CONTROLLER_TF_RT_BROADCASTER_H_
#define RM_BASE_INCLUDE_RM_BASE_ROBOT_STATE_CONTROLLER_TF_RT_BROADCASTER_H_
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <realtime_tools/realtime_publisher.h>

namespace robot_state_controller {
class TfRtBroadcaster {
 public:
  TfRtBroadcaster() = default;
  virtual void init(ros::NodeHandle &root_nh);
  virtual void sendTransform(const geometry_msgs::TransformStamped &transform);
  virtual void sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms);
 protected:
  ros::NodeHandle node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> realtime_pub_{};
};

class StaticTfRtBroadcaster : public TfRtBroadcaster {
 public:
  void init(ros::NodeHandle &root_nh) override;
  void sendTransform(const geometry_msgs::TransformStamped &transform) override;
  void sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) override;
 private:
  tf2_msgs::TFMessage net_message_{};
};

}
#endif //RM_BASE_INCLUDE_RM_BASE_ROBOT_STATE_CONTROLLER_TF_RT_BROADCASTER_H_
