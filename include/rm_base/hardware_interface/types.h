//
// Created by qiayuan on 1/20/21.
//

#ifndef RM_BASE_INCLUDE_RM_BASE_TYPES_H_
#define RM_BASE_INCLUDE_RM_BASE_TYPES_H_
#include <string>
#include <lp_filter.h>
#include <unordered_map>

namespace rm_base {
struct ActCoeff {
  double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out,
      act2pos_offset, act2vel_offset, act2effort_offset, kp2act, kd2act; // for MIT Cheetah motor
};

struct ActData {
  std::string type{};
  double pos{}, vel{}, effort{}, cmd_pos{}, cmd_vel{}, cmd_effort{};
  // for RoboMaseter encoder
  int64_t q_circle{};
  uint16_t q_last{};
  uint8_t temp{};
  LowPassFilter *lp_filter{};
};

struct ImuData {
  std::string type{};
  double ori[4]{};
  double ori_cov[9]{};
  double angular_vel[3]{};
  double angular_vel_cov[9]{};
  double linear_acc[3]{};
  double linear_acc_cov[9]{};
};

struct CanDataPtr {
  std::unordered_map<std::string, ActCoeff> *type2act_coeffs_{};
  std::unordered_map<int, ActData> *id2act_data_{};
  std::unordered_map<int, ImuData> *id2imu_data_{};
};
}

#endif //RM_BASE_INCLUDE_RM_BASE_TYPES_H_
