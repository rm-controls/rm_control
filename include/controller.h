//
// Created by qiayuan on 7/4/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_CONTROLLER_H_
#define SRC_RM_COMMON_INCLUDE_CONTROLLER_H_
#include "math_utilities.h"
#include <string>

template<typename T>
class JointCtrl {
 public:
  JointCtrl() = default;

  virtual void setK(T kp, T kd) { kp_ = kp, kd_ = kd; }
  virtual void setDes(T q_des, T qd_des, T ff) {
    q_des_ = q_des;
    qd_des_ = qd_des;
    ff_ = ff;
  };
  virtual void setK(const std::string &name, T k) {}

  virtual void input(T q, T qd, T effect) {};
  T output() { return out_; }
 protected:
  T kp_{}, kd_{}, q_des_{}, qd_des_{}, ff_{};
  T out_{};
};

template<typename T>
class PDCtrl : public JointCtrl<T> {
 public:
  void input(T q, T qd, T effect) override {
    this->out_ = this->kp_ * (this->q_des_ - q) +
        this->kd_ * (this->qd_des_ - qd) + this->ff_;
  }
};

template<typename T>
class SpeedCtrl : public JointCtrl<T> {
 public:
  void setK(const std::string &name, T k) override {
    if (name == "dt")
      dt_ = k;
  }
  void input(T q, T qd, T effect) override {
    if (dt_ == 0.)
      this->out_ = 0.;
    else
      this->out_ =
          this->kp_ * (this->qd_des_ - qd) + this->kd_ * (qd_last_ - qd)
              + this->ff_;
    qd_last_ = qd;
  }
 private:
  T qd_last_{}, dt_{};
};

#endif //SRC_RM_COMMON_INCLUDE_CONTROLLER_H_
