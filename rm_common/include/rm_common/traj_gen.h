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
// Created by qiayuan on 3/21/20.
//

#pragma once

#include <cmath>
#include <string>
#include "math_utilities.h"

template <typename T>
class RampTraj
{
private:
  T start_{};   // Start pos and velocity
  T target_{};  // End pos and velocity
  T time_total_{};
  T time_acc_{};
  T time_start_{};

  T acc_{};
  T speed_{};  // When in uniform speed
  T a0_, b0_, c0_;
  T a1_, b1_, c1_;

public:
  RampTraj() = default;
  void setLimit(T max_acc)
  {
    acc_ = max_acc;
  }
  void setState(T start, T end, T time_now)
  {
    start_ = start;
    target_ = end;
    time_start_ = time_now;
  };
  bool isReach(T t)
  {
    return (t >= time_total_ + time_start_);
  }

  bool calc(T t)
  {
    if ((target_ - start_) < 0)
    {
      acc_ = -acc_;
    }
    time_total_ = t;
    if (abs(acc_)  // if distant is too small
        < abs(4. * (target_ - start_)) / (t * t))
    {
      return false;
    }
    time_acc_ = time_total_ / 2. -
                sqrt(acc_ * acc_ * time_total_ * time_total_ - 4. * acc_ * (target_ - start_)) / abs(2. * acc_);
    a0_ = start_;
    b0_ = 0.;
    c0_ = 0.5 * acc_;
    a1_ = target_ - (acc_ * time_total_ * time_total_) / 2.;
    b1_ = acc_ * time_total_;
    c1_ = -0.5 * acc_;
    speed_ = b0_ + 2. * c0_ * time_acc_;
    return true;
  };

  T getPos(T t)
  {
    t -= time_start_;
    if (t < 0.)
      return start_;
    else if (t > time_total_)
      return target_;

    if (t <= time_total_)
    {
      if (t < time_acc_)
        return a0_ + b0_ * t + c0_ * t * t;
      else if (t < time_total_ - time_acc_)
        return speed_ * (t - time_acc_) + a0_ + c0_ * (time_acc_ * time_acc_);
      else
        return a1_ + b1_ * t + c1_ * t * t;
    }
    else
      return target_;
  }

  T getVel(T t)
  {
    t -= time_start_;
    if (t < 0. || t > time_total_)
      return 0;

    if (t <= time_total_)
    {
      if (t < time_acc_)
        return b0_ + 2. * c0_ * t;
      else if (t < time_total_ - time_acc_)
        return speed_;
      else
        return b1_ + 2. * c1_ * t;
    }
    else
      return 0.;
  }

  T getAcc(T t)
  {
    t -= time_start_;
    if (t < 0. || t > time_total_)
      return 0;

    if (t <= time_total_)
    {
      if (t < time_acc_)
        return 2. * c0_;
      else if (t < time_total_ - time_acc_)
        return 0;
      else
        return 2. * c1_;
    }
    else
      return 0.;
  }
};

// actually it is a controller
// ref: https://build-its-inprogress.blogspot.com/2017/12/controls-ramblings-how-to-get-from.html
template <typename T>
class MinTimeTraj
{
private:
  T target_{};
  T inertia_{};
  T max_tau_{};
  T tolerance_{};
  bool is_reach_{};

public:
  MinTimeTraj() = default;
  void setLimit(T max_tau, T inertia, T tolerance)
  {
    max_tau_ = max_tau;
    inertia_ = inertia;
    tolerance_ = tolerance;
  }

  void setTarget(T target)
  {
    target_ = target;
    is_reach_ = false;
  }
  bool isReach()
  {
    return is_reach_;
  }
  T getTau(T pos, T vel)
  {
    T dx = pos - target_;
    if (std::abs(dx) > tolerance_)
      return max_tau_ * sgn(-max_tau_ * dx - 0.5 * inertia_ * vel * std::abs(vel));
    else
    {
      is_reach_ = true;
      return 0;
    }
  }
};
