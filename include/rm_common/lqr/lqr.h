//
// Created by chenzheng on 3/20/21.
//

#ifndef RM_COMMON_FILTERS_LQR_H
#define RM_COMMON_FILTERS_LQR_H
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SparseLU>

template<typename T>
class Lqr {
 public:
  template<typename TA, typename TB, typename TQ, typename TR>
  Lqr(const Eigen::MatrixBase<TA> &A,
      const Eigen::MatrixBase<TB> &B,
      const Eigen::MatrixBase<TQ> &Q,
      const Eigen::MatrixBase<TR> &R) :
      a_(A), b_(B), q_(Q), r_(R) {
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

  }

  ~Lqr() = default;

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> getK() {
    Eigen::SparseLU<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, Eigen::COLAMDOrdering<int>> solver;
    solver.isSymmetric();
    return k_;
  }
 private:
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> a_, b_, q_, r_, k_;

};

#endif //RM_COMMON_LQR_LQR_H
