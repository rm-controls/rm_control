//
// Created by chenzheng on 3/20/21.
//

#ifndef RM_COMMON_FILTERS_LQR_H
#define RM_COMMON_FILTERS_LQR_H
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SparseLU>
#include "eigen_types.h"

template<typename T>
class Lqr {
 public:

  template<typename TA, typename TB, typename TQ, typename TR>
  Lqr(const Eigen::MatrixBase<TA> &A, const Eigen::MatrixBase<TB> &B,
      const Eigen::MatrixBase<TQ> &Q, const Eigen::MatrixBase<TR> &R, T eps = 1e-15) :
      a_(A), b_(B), q_(Q), r_(R), eps_(eps) {
    //check A
    static_assert(TA::RowsAtCompileTime == TA::ColsAtCompileTime, "lqr: A should be square matrix");
    //check B
    static_assert(TB::RowsAtCompileTime == TA::RowsAtCompileTime, "lqr: B rows should be equal to A rows");
    //check Q
    static_assert(TQ::RowsAtCompileTime == TA::RowsAtCompileTime && TQ::ColsAtCompileTime == TA::ColsAtCompileTime,
                  "lqr: The rows and columns of Q should be equal to A");
    //check R
    static_assert(TR::RowsAtCompileTime == TB::ColsAtCompileTime && TR::ColsAtCompileTime == TB::ColsAtCompileTime,
                  "lqr: The rows and columns of R should be equal to the cols of B");

    k_.resize(TB::ColsAtCompileTime, TB::RowsAtCompileTime);
    k_.setZero();
  }

  bool computeK() {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > q_solver(q_);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > r_solver(r_);
    if (q_solver.info() != Eigen::Success || r_solver.info() != Eigen::Success)
      return false;
    // q (r) must be symmetric positive (semi) definite
    for (int i = 0; i < q_solver.eigenvalues().cols(); ++i) {
      if (q_solver.eigenvalues()[i] < 0)
        return false;
    }
    for (int i = 0; i < r_solver.eigenvalues().cols(); ++i) {
      if (r_solver.eigenvalues()[i] <= 0)
        return false;
    }
//    if (!isSymmetric(q_) || !isSymmetric(r_))
//      return false;

    // precompute as much as possible
    DMat<T> a_t = a_.transpose();
    DMat<T> b_t = b_.transpose();
    // initialize P with Q
    DMat<T> p = q_;
    // iterate until P converges
    unsigned int i = 0;
    DMat<T> p_old = p;
    while (true) {
      i++;
      // compute new P
      p = a_t * p * a_ -
          a_t * p * b_ * (r_ + b_t * p * b_).inverse() * b_t * p * a_ + q_;
      // update delta
      DMat<T> delta = p - p_old;
      if (std::fabs(delta.maxCoeff()) < eps_) {
        std::cout << "Number of iterations until convergence: " << i << std::endl;
        break;
      }
      p_old = p;
    }
    k_ = (r_ + b_t * p * b_).inverse() * (b_t * p * a_);
    return true;
  }

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getK() {
    return k_;
  }

 private:
  bool isSymmetric(DMat<T> m) {
    for (int i = 0; i < m.rows() - 1; ++i) {
      for (int j = i + 1; j < m.cols(); ++j) {
        if (m[i][j - i] != m[j - i][i])
          return false;
      }
    }
    return true;
  }

  DMat<T> a_, b_, q_, r_, k_;
  T eps_;
};

#endif //RM_COMMON_LQR_LQR_H
