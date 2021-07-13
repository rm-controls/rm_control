//
// Created by kiana on 2021/3/22.
//

#include "rm_common/decision/target_cost_function.h"
namespace rm_common {

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh, const Referee &referee) :
    referee_{referee}, optimal_id_(0) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  if (!cost_nh.getParam("k_pos", k_pos_))
    ROS_ERROR("K position no defined (namespace: %s)", cost_nh.getNamespace().c_str());
  if (!cost_nh.getParam("k_vel", k_vel_))
    ROS_ERROR("K velocity  no defined (namespace: %s)", cost_nh.getNamespace().c_str());
  if (!cost_nh.getParam("k_hp", k_hp_))
    ROS_ERROR("K velocity no defined (namespace: %s)", cost_nh.getNamespace().c_str());
  if (!cost_nh.getParam("k_freq", k_freq_))
    ROS_ERROR("K frequency no defined (namespace: %s)", cost_nh.getNamespace().c_str());
  if (!cost_nh.getParam("timeout", timeout_))
    ROS_ERROR("Timeout no defined (namespace: %s)", cost_nh.getNamespace().c_str());
}

double TargetCostFunction::costFunction(const rm_msgs::TrackDataArray &track_data_array, bool only_attack_base) {
  double optimal_cost = 1e9;
  for (const auto &track:track_data_array.tracks) {
    if (id2target_states_.find(track.id) == id2target_states_.end())
      id2target_states_.insert(std::make_pair(track.id, TargetState{.id= track.id}));
    TargetState &target_state = id2target_states_.find(track.id)->second;
    target_state.pos_x = track.camera2detection.x;
    target_state.pos_y = track.camera2detection.y;
    target_state.pos_z = track.camera2detection.z;
    // TODO Add Vel to msg
    target_state.vel_x = 0.;
    target_state.vel_y = 0.;
    target_state.vel_z = 0.;
    target_state.last_receive_ = track_data_array.header.stamp;
  }
  for (const auto &target_state: id2target_states_) {
    if ((ros::Time::now() - target_state.second.last_receive_).toSec() > timeout_)
      continue;
    double cost = costFunction(target_state.second, only_attack_base);
    if (cost < optimal_cost) {
      optimal_cost = cost;
      if (optimal_id_ != target_state.first) last_switch_target_ = ros::Time::now();
      optimal_id_ = target_state.first;
    }
  }
  return optimal_cost;
}

double TargetCostFunction::costFunction(const TargetState &target_state, bool only_attack_base) {
  if (only_attack_base) return 0.;
  double distance = sqrt(pow(target_state.pos_x, 2) + pow(target_state.pos_y, 2) + pow(target_state.pos_z, 2));
  // TODO Finish Vel cost
  double velocity = 0.;
  double hp_cost = 0.;
  if (referee_.is_online_) {
    if (referee_.referee_data_.game_robot_status_.robot_id_ <= RED_RADAR) {  // The enemy color is blue
      if (target_state.id == 1) hp_cost = referee_.referee_data_.game_robot_hp_.blue_1_robot_hp_;
      else if (target_state.id == 2) hp_cost = referee_.referee_data_.game_robot_hp_.blue_2_robot_hp_;
      else if (target_state.id == 3) hp_cost = referee_.referee_data_.game_robot_hp_.blue_3_robot_hp_;
      else if (target_state.id == 4) hp_cost = referee_.referee_data_.game_robot_hp_.blue_4_robot_hp_;
      else if (target_state.id == 5) hp_cost = referee_.referee_data_.game_robot_hp_.blue_5_robot_hp_;
      else if (target_state.id == 7) hp_cost = referee_.referee_data_.game_robot_hp_.blue_7_robot_hp_;
    } else {    // The enemy color is red
      if (target_state.id == 1) hp_cost = referee_.referee_data_.game_robot_hp_.red_1_robot_hp_;
      else if (target_state.id == 2) hp_cost = referee_.referee_data_.game_robot_hp_.red_2_robot_hp_;
      else if (target_state.id == 3) hp_cost = referee_.referee_data_.game_robot_hp_.red_3_robot_hp_;
      else if (target_state.id == 4) hp_cost = referee_.referee_data_.game_robot_hp_.red_4_robot_hp_;
      else if (target_state.id == 5) hp_cost = referee_.referee_data_.game_robot_hp_.red_5_robot_hp_;
      else if (target_state.id == 7) hp_cost = referee_.referee_data_.game_robot_hp_.red_7_robot_hp_;
    }
  }
  double frequency =
      (target_state.id == optimal_id_ || optimal_id_ == 0) ? 0. : 1. / (ros::Time::now() - last_switch_target_).toSec();
  return k_pos_ * distance + k_vel_ * velocity + k_hp_ * hp_cost + k_freq_ * frequency; //TODO velocity
}

}