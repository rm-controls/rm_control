/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 4/3/20.
//

#pragma once

#include <iostream>
#include "rm_common/eigen_types.h"

template <typename T>
class KalmanFilter
{
public:
  template <typename TA, typename TB, typename TH, typename TQ, typename TR>
  KalmanFilter(const Eigen::MatrixBase<TA>& A, const Eigen::MatrixBase<TB>& B, const Eigen::MatrixBase<TH>& H,
               const Eigen::MatrixBase<TQ>& Q, const Eigen::MatrixBase<TR>& R)
    : A_(A), B_(B), H_(H), Q_(Q), R_(R), m_(TH::RowsAtCompileTime), n_(TA::RowsAtCompileTime), inited(false)
  {
    // Check dimension
    // ref:http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/MatrixKalman.html
    static_assert(TA::RowsAtCompileTime == TA::ColsAtCompileTime, "A should be square matrix");
    static_assert(TA::RowsAtCompileTime == TH::ColsAtCompileTime, "H columns should be equal to A columns");
    static_assert(TB::RowsAtCompileTime == TA::ColsAtCompileTime, "B rows should be equal to A columns ");
    static_assert(TQ::RowsAtCompileTime == TA::ColsAtCompileTime && TQ::ColsAtCompileTime == TA::ColsAtCompileTime,
                  "The rows and columns of Q should be equal to the columns of A");
    static_assert(TR::RowsAtCompileTime == TH::RowsAtCompileTime && TR::ColsAtCompileTime == TH::RowsAtCompileTime,
                  "The rows and columns of Q should be equal to the rows of H");
    x_.resize(n_);
    P_.resize(n_, n_);
    I_.resize(n_, n_);
    I_.setIdentity();
  }

  ~KalmanFilter() = default;

  template <typename T1>
  void clear(const Eigen::MatrixBase<T1>& x)
  {
    x_ = x;
    inited = true;
    K_ = DMat<T>::Zero(n_, m_);
    P_ = DMat<T>::Zero(n_, m_);
    P_new_ = DMat<T>::Zero(n_, n_);
  }

  template <typename T1>
  void update(const Eigen::MatrixBase<T1>& z)
  {
    update(z, R_);
  };

  template <typename T1, typename T2>
  void update(const Eigen::MatrixBase<T1>& z, const Eigen::MatrixBase<T2>& R)
  {
    if (!inited)
      return;  // TODO: add assert
    // update R_
    R_ = R;
    // update
    K_ = P_new_ * H_.transpose() * ((H_ * P_new_ * H_.transpose() + R_).inverse());
    x_ = x_ + K_ * (z - H_ * x_);
    P_ = (I_ - K_ * H_) * P_new_;
  };

  template <typename T1>
  void predict(const Eigen::MatrixBase<T1>& u)
  {
    predict(u, Q_);
  }

  template <typename T1, typename T2>
  void predict(const Eigen::MatrixBase<T1>& u, const Eigen::MatrixBase<T2>& Q)
  {
    if (!inited)
      return;  // TODO: add assert
    // update Q_
    Q_ = Q;
    // predict
    x_ = A_ * x_ + B_ * u;
    P_new_ = A_ * P_ * A_.transpose() + Q_;
  }

  DVec<T> getState()
  {
    return x_;
  }

private:
  DMat<T> A_, B_, H_, I_;
  DMat<T> Q_, R_, P_, P_new_, K_;
  DVec<T> x_;
  const int m_, n_;  // dimension
  bool inited;
};
