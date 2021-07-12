//
// Created by qiayuan on 5/19/21.
//

#ifndef RM_COMMON_HEAT_LIMIT_H_
#define RM_COMMON_HEAT_LIMIT_H_
#include <rm_common/referee/referee.h>
namespace rm_common {
class HeatLimit {
 public:
  HeatLimit(ros::NodeHandle &nh, const Referee &referee) : referee_(referee) {
    if (!nh.getParam("expect_shoot_frequency_1", expect_shoot_frequency_1_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("expect_shoot_frequency_2", expect_shoot_frequency_2_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("expect_shoot_frequency_3", expect_shoot_frequency_3_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("burst_shoot_frequency", burst_shoot_frequency_))
      ROS_ERROR("Expect shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("safe_shoot_frequency", safe_shoot_frequency_))
      ROS_ERROR("Safe shoot frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("heat_coeff", heat_coeff_))
      ROS_ERROR("Safe shoot heat coeff frequency no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("type", type_))
      ROS_ERROR("Shooter type no defined (namespace: %s)", nh.getNamespace().c_str());
    if (type_ == "ID1_42MM")
      bullet_heat_ = 100.;
    else
      bullet_heat_ = 10.;
  }

  double getHz() const {
    if (burst_flag_) return shoot_frequency_;
    if (!referee_.is_online_) return safe_shoot_frequency_;
    double cooling_limit{}, cooling_rate{}, cooling_heat{};
    if (type_ == "ID1_17MM") {
      cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_;
      cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_rate_;
      cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_;
    } else if (type_ == "ID2_17MM") {
      cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_limit_;
      cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_rate_;
      cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id_2_17_mm_cooling_heat_;
    } else if (type_ == "ID1_42MM") {
      cooling_limit = referee_.referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_;
      cooling_rate = referee_.referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_rate_;
      cooling_heat = referee_.referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_;
    }

    if (cooling_limit - cooling_heat < bullet_heat_)
      return 0.0;
    else if (cooling_limit - cooling_heat == bullet_heat_)
      return cooling_rate / bullet_heat_;
    else if (cooling_limit - cooling_heat <= bullet_heat_ * heat_coeff_)
      return (cooling_limit - cooling_heat) / (bullet_heat_ * heat_coeff_)
          * (shoot_frequency_ - cooling_rate / bullet_heat_) + cooling_rate / bullet_heat_;
    else
      return shoot_frequency_;
  }

  int getSpeedLimit() {
    if (type_ == "ID1_17MM")
      switch (referee_.referee_data_.game_robot_status_.shooter_id_1_17_mm_speed_limit_) {
        case 15: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
        case 18: return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
        case 30: return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
        default: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;  // Safety speed
      }
    else if (type_ == "ID2_17MM")
      switch (referee_.referee_data_.game_robot_status_.shooter_id_2_17_mm_speed_limit_) {
        case 15: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
        case 18: return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
        case 30: return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
        default: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;  // Safety speed
      }
    else if (type_ == "ID1_42MM")
      switch (referee_.referee_data_.game_robot_status_.shooter_id_1_42_mm_speed_limit_) {
        case 10: return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;
        case 16: return rm_msgs::ShootCmd::SPEED_16M_PER_SECOND;
        default: return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;  // Safety speed
      }
    return -1;    // TODO unsafe!
  }

  void updateExpectShootFrequency() {
    if (referee_.referee_data_.game_robot_status_.robot_level_ == 1)
      shoot_frequency_ = expect_shoot_frequency_1_;
    else if (referee_.referee_data_.game_robot_status_.robot_level_ == 2)
      shoot_frequency_ = expect_shoot_frequency_2_;
    else if (referee_.referee_data_.game_robot_status_.robot_level_ == 3)
      shoot_frequency_ = expect_shoot_frequency_3_;
    else if (burst_flag_)
      shoot_frequency_ = burst_shoot_frequency_;
    else
      shoot_frequency_ = safe_shoot_frequency_;
  }

  void setMode(bool burst_flag) { burst_flag_ = burst_flag; }
  bool getMode() { return burst_flag_; }

 private:
  std::string type_{};
  const Referee &referee_;
  double bullet_heat_, safe_shoot_frequency_{}, heat_coeff_{}, shoot_frequency_{}, expect_shoot_frequency_1_{},
      expect_shoot_frequency_2_{}, expect_shoot_frequency_3_{}, burst_shoot_frequency_{};
  bool burst_flag_ = false;
};

}
#endif //RM_COMMON_HEAT_LIMIT_H_
