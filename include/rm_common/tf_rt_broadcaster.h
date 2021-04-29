//
// Created by qiayuan on 1/3/21.
//

#ifndef RM_COMMON_TF_RT_BROADCASTER_H
#define RM_COMMON_TF_RT_BROADCASTER_H
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <realtime_tools/realtime_publisher.h>

namespace rm_common {
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
#endif // RM_COMMON_TF_RT_BROADCASTER_H
