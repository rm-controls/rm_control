//
// Created by qiayuan on 4/3/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H_
#define SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H_
#include <iostream>
#include "cpp_types.h"

template<typename T>
class KalmanFilter {
 public:
  template<typename TA, typename TB, typename TH, typename TQ, typename TR>
  KalmanFilter(const Eigen::MatrixBase<TA> &A,
               const Eigen::MatrixBase<TB> &B,
               const Eigen::MatrixBase<TH> &H,
               const Eigen::MatrixBase<TQ> &Q,
               const Eigen::MatrixBase<TR> &R) :
      A_(A), B_(B), H_(H), Q_(Q), R_(R),
      m_(TH::RowsAtCompileTime), n_(TA::RowsAtCompileTime),
      inited(false) {
    // Check dimension
    // ref:http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/MatrixKalman.html
    static_assert(TA::RowsAtCompileTime == TA::ColsAtCompileTime,
                  "A should be square matrix");
    static_assert(TA::RowsAtCompileTime == TH::ColsAtCompileTime,
                  "H columns should be equal to A columns");
    static_assert(TB::RowsAtCompileTime == TA::ColsAtCompileTime,
                  "B rows should be equal to A columns ");
    static_assert(TQ::RowsAtCompileTime == TA::ColsAtCompileTime &&
                      TQ::ColsAtCompileTime == TA::ColsAtCompileTime,
                  "The rows and columns of Q should be equal to the columns of A");
    static_assert(TR::RowsAtCompileTime == TH::RowsAtCompileTime &&
                      TR::ColsAtCompileTime == TH::RowsAtCompileTime,
                  "The rows and columns of Q should be equal to the rows of H");
    x_.resize(n_);
    P_.resize(n_, n_);
    I_.resize(n_, n_);
    I_.setIdentity();
  }

  ~KalmanFilter() = default;

  template<typename T1>
  void clear(const Eigen::MatrixBase<T1> &x) {
    x_ = x;
    inited = true;
    K_ = DMat<T>::Zero(n_, m_);
    P_ = DMat<T>::Zero(n_, m_);
    P_new_ = DMat<T>::Zero(n_, n_);
  }

  template<typename T1>
  void update(const Eigen::MatrixBase<T1> &z) {
    if (!inited)
      return; //TODO: add assert
    K_ = P_new_ * H_.transpose()
        * ((H_ * P_new_ * H_.transpose() + R_).inverse());
    x_ = x_ + K_ * (z - H_ * x_);
    P_ = (I_ - K_ * H_) * P_new_;
  };

  template<typename T1>
  void predict(const Eigen::MatrixBase<T1> &u) {
    if (!inited)
      return; //TODO: add assert
    //predict
    x_ = A_ * x_ + B_ * u;
    P_new_ = A_ * P_ * A_.transpose() + Q_;
  }

  DVec<T> getState() {
    return x_;
  }

 private:
  DMat<T> A_, B_, H_, I_;
  DMat<T> Q_, R_, P_, P_new_, K_;
  DVec<T> x_;
  const int m_, n_;//dimension
  bool inited;
};

#endif //SRC_RM_COMMON_INCLUDE_KALMAN_FILTER_H_
