//
// Created by xiezd on 3/25/26.
//

#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include "eigen_types.h"

#define RLS_ASSERT(condition, message)                                                                                 \
  if (!(condition))                                                                                                    \
  throw std::invalid_argument(message)

template <typename T>
class Rls
{
public:
  Rls(int xSize, int ySize, T lambda, T pInit) : xSize_(xSize), ySize_(ySize), lambda_(lambda)
  {
    RLS_ASSERT(xSize_ > 0, "rls: xSize should be positive");
    RLS_ASSERT(ySize_ > 0, "rls: ySize should be positive");
    RLS_ASSERT(std::abs(lambda_) > std::numeric_limits<T>::epsilon(), "rls: lambda should be non-zero");

    x_.resize(xSize_, 1);
    x_.setZero();
    xt_.resize(1, xSize_);
    xt_.setZero();

    w_.resize(xSize_, ySize_);
    w_.setZero();
    output_.resize(xSize_, ySize_);
    output_.setZero();
    k_.resize(xSize_, 1);
    k_.setZero();
    kNumerator_.resize(xSize_, 1);
    kNumerator_.setZero();

    p_.resize(xSize_, xSize_);
    p_.setIdentity();
    p_ *= pInit;

    y_.resize(ySize_, 1);
    y_.setZero();
    u_.resize(ySize_, 1);
    u_.setZero();
    e_.resize(ySize_, 1);
    e_.setZero();
  }

  template <typename TX>
  bool setX(const Eigen::MatrixBase<TX>& x)
  {
    static_assert(TX::ColsAtCompileTime == 1 || TX::ColsAtCompileTime == Eigen::Dynamic,
                  "rls: X should be a column vector");
    RLS_ASSERT(x.rows() == xSize_ && x.cols() == 1, "rls: X size mismatch");
    x_ = x;
    return true;
  }

  template <typename TY>
  bool setY(const Eigen::MatrixBase<TY>& y)
  {
    static_assert(TY::ColsAtCompileTime == 1 || TY::ColsAtCompileTime == Eigen::Dynamic,
                  "rls: Y should be a column vector");
    RLS_ASSERT(y.rows() == ySize_ && y.cols() == 1, "rls: Y size mismatch");
    y_ = y;
    return true;
  }

  bool setY(T y)
  {
    RLS_ASSERT(ySize_ == 1, "rls: setY(scalar) requires ySize==1");
    y_(0, 0) = y;
    return true;
  }

  template <typename TW>
  bool setW(const Eigen::MatrixBase<TW>& w)
  {
    RLS_ASSERT(w.rows() == xSize_ && w.cols() == ySize_, "rls: W size mismatch");
    w_ = w;
    return true;
  }

  template <typename TU>
  bool setU(const Eigen::MatrixBase<TU>& u)
  {
    static_assert(TU::ColsAtCompileTime == 1 || TU::ColsAtCompileTime == Eigen::Dynamic,
                  "rls: U should be a column vector");
    RLS_ASSERT(u.rows() == ySize_ && u.cols() == 1, "rls: U size mismatch");
    u_ = u;
    return true;
  }

  bool setU(T u)
  {
    RLS_ASSERT(ySize_ == 1, "rls: setU(scalar) requires ySize==1");
    u_(0, 0) = u;
    return true;
  }

  bool update()
  {
    xt_ = x_.transpose();
    u_ = w_.transpose() * x_;
    e_ = y_ - u_;

    kNumerator_ = p_ * x_;
    T denominator = lambda_ + (xt_ * p_ * x_)(0, 0);
    RLS_ASSERT(std::abs(denominator) > std::numeric_limits<T>::epsilon(),
               "rls: denominator too small, numerical instability");

    k_ = kNumerator_ / denominator;
    output_ = w_ + k_ * e_.transpose();
    w_ = output_;

    p_ = (p_ - (p_ * x_ * xt_ * p_) / denominator) / lambda_;
    return true;
  }

  DMat<T> getW() const
  {
    return w_;
  }

  DMat<T> getP() const
  {
    return p_;
  }

  DVec<T> getError() const
  {
    return e_;
  }

  DVec<T> getEstimate() const
  {
    return u_;
  }

private:
  int xSize_;
  int ySize_;
  T lambda_;

  DVec<T> x_;
  DMat<T> xt_;
  DMat<T> w_;
  DMat<T> output_;
  DVec<T> k_;
  DVec<T> kNumerator_;
  DMat<T> p_;

  DVec<T> y_;
  DVec<T> u_;
  DVec<T> e_;
};
