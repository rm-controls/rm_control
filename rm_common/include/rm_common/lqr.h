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
// Created by chenzheng on 3/20/21.
//

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "eigen_types.h"

template <typename T>
class Lqr
{
public:
  template <typename TA, typename TB, typename TQ, typename TR>
  Lqr(const Eigen::MatrixBase<TA>& A, const Eigen::MatrixBase<TB>& B, const Eigen::MatrixBase<TQ>& Q,
      const Eigen::MatrixBase<TR>& R)
    : a_(A), b_(B), q_(Q), r_(R)
  {
    // check A
    static_assert(TA::RowsAtCompileTime == TA::ColsAtCompileTime, "lqr: A should be square matrix");
    // check B
    static_assert(TB::RowsAtCompileTime == TA::RowsAtCompileTime, "lqr: B rows should be equal to A rows");
    // check Q
    static_assert(TQ::RowsAtCompileTime == TA::RowsAtCompileTime && TQ::ColsAtCompileTime == TA::ColsAtCompileTime,
                  "lqr: The rows and columns of Q should be equal to A");
    // check R
    static_assert(TR::RowsAtCompileTime == TB::ColsAtCompileTime && TR::ColsAtCompileTime == TB::ColsAtCompileTime,
                  "lqr: The rows and columns of R should be equal to the cols of B");

    k_.resize(TB::ColsAtCompileTime, TB::RowsAtCompileTime);
    k_.setZero();
  }

  bool solveRiccatiArimotoPotter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q,
                                 const Eigen::MatrixXd& R, Eigen::MatrixXd& P)
  {
    int dim_x = A.rows();

    // set Hamilton matrix
    Eigen::MatrixXd ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> eigs(ham);

    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    int j = 0;
    for (int i = 0; i < 2 * dim_x; ++i)
    {
      if (eigs.eigenvalues()[i].real() < 0.)
      {
        eigvec.col(j) = eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
        ++j;
      }
    }

    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd vs_1, vs_2;
    vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (vs_2 * vs_1.inverse()).real();

    return true;
  }

  bool computeK()
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > q_solver(q_);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > r_solver(r_);
    if (q_solver.info() != Eigen::Success || r_solver.info() != Eigen::Success)
      return false;
    // q (r) must be symmetric positive (semi) definite
    for (int i = 0; i < q_solver.eigenvalues().cols(); ++i)
    {
      if (q_solver.eigenvalues()[i] < 0)
        return false;
    }
    for (int i = 0; i < r_solver.eigenvalues().cols(); ++i)
    {
      if (r_solver.eigenvalues()[i] <= 0)
        return false;
    }
    if (!isSymmetric(q_) || !isSymmetric(r_))
      return false;

    DMat<T> p;
    if (solveRiccatiArimotoPotter(a_, b_, q_, r_, p))
      k_ = r_.inverse() * (b_.transpose() * p.transpose());
    return true;
  }

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getK()
  {
    return k_;
  }

private:
  bool isSymmetric(DMat<T> m)
  {
    for (int i = 0; i < m.rows() - 1; ++i)
    {
      for (int j = i + 1; j < m.cols(); ++j)
      {
        if (m(i, j - i) != m(j - i, i))
          return false;
      }
    }
    return true;
  }

  DMat<T> a_, b_, q_, r_, k_;
};
