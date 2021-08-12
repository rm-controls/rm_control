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

#include "rm_common/filters/filters.h"
#include "rm_common/math_utilities.h"
#include <cmath>
#include <cstring>

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(int num_data) : num_data_(num_data), idx_(0), sum_(0.0)
{
  buffer_ = new T[num_data_];
  memset((void*)buffer_, 0.0, sizeof(T) * num_data_);
}

template <typename T>
void MovingAverageFilter<T>::input(T input_value)
{
  sum_ -= buffer_[idx_];
  sum_ += input_value;
  buffer_[idx_] = input_value;
  ++idx_;
  idx_ %= num_data_;
}

template <typename T>
T MovingAverageFilter<T>::output()
{
  return sum_ / num_data_;
}

template <typename T>
void MovingAverageFilter<T>::clear()
{
  sum_ = 0.0;
  memset((void*)buffer_, 0.0, sizeof(T) * num_data_);
}

template <typename T>
MovingAverageFilter<T>::~MovingAverageFilter()
{
  delete[] buffer_;
}

template class MovingAverageFilter<double>;
template class MovingAverageFilter<float>;

/*============================================================================*/

template <typename T>
ButterworthFilter<T>::ButterworthFilter(int num_sample, T dt, T cutoff_frequency)
{
  mNumSample_ = num_sample;
  mDt_ = dt;
  mCutoffFreq_ = cutoff_frequency;

  mpBuffer_ = new T[num_sample];
  memset((void*)mpBuffer_, 0, sizeof(T) * num_sample);

  mCurIdx_ = 0;
}

template <typename T>
ButterworthFilter<T>::~ButterworthFilter()
{
  delete[] mpBuffer_;
}

template <typename T>
void ButterworthFilter<T>::input(T input_value)
{
  int j;
  T sqrt_2 = sqrt(2.);
  T value = 0;
  for (j = mNumSample_ - 2; j >= 0; j--)
  {
    mpBuffer_[j + 1] = mpBuffer_[j];
  }

  mpBuffer_[0] = input_value;
  for (j = 0; j < mNumSample_; j++)
  {
    T t = (T)j * mDt_;
    value += sqrt_2 / mCutoffFreq_ * mpBuffer_[j] * exp(-1. / sqrt_2 * t) * sin(mCutoffFreq_ / sqrt_2 * t) * mDt_;
  }
  mValue_ = value;
}

template <typename T>
T ButterworthFilter<T>::output()
{
  return mValue_;
}

template <typename T>
void ButterworthFilter<T>::clear()
{
  for (int i(0); i < mNumSample_; ++i)
  {
    mpBuffer_[i] = 0.0;
  }
}

template class ButterworthFilter<double>;
template class ButterworthFilter<float>;

/*============================================================================*/

template <typename T>
DigitalLpFilter<T>::DigitalLpFilter(T w_c, T t_s)
{
  Lpf_in_prev_[0] = Lpf_in_prev_[1] = 0;
  Lpf_out_prev_[0] = Lpf_out_prev_[1] = 0;
  Lpf_in1_ = 0, Lpf_in2_ = 0, Lpf_in3_ = 0, Lpf_out1_ = 0, Lpf_out2_ = 0;
  float den = 2500 * t_s * t_s * w_c * w_c + 7071 * t_s * w_c + 10000;

  Lpf_in1_ = 2500 * t_s * t_s * w_c * w_c / den;
  Lpf_in2_ = 5000 * t_s * t_s * w_c * w_c / den;
  Lpf_in3_ = 2500 * t_s * t_s * w_c * w_c / den;
  Lpf_out1_ = -(5000 * t_s * t_s * w_c * w_c - 20000) / den;
  Lpf_out2_ = -(2500 * t_s * t_s * w_c * w_c - 7071 * t_s * w_c + 10000) / den;
}

template <typename T>
DigitalLpFilter<T>::~DigitalLpFilter() = default;

template <typename T>
void DigitalLpFilter<T>::input(T lpf_in)
{
  lpf_out_ = Lpf_in1_ * lpf_in + Lpf_in2_ * Lpf_in_prev_[0] + Lpf_in3_ * Lpf_in_prev_[1] +  // input component
             Lpf_out1_ * Lpf_out_prev_[0] + Lpf_out2_ * Lpf_out_prev_[1];                   // output component
  Lpf_in_prev_[1] = Lpf_in_prev_[0];
  Lpf_in_prev_[0] = lpf_in;
  Lpf_out_prev_[1] = Lpf_out_prev_[0];
  Lpf_out_prev_[0] = lpf_out_;
}

template <typename T>
T DigitalLpFilter<T>::output()
{
  return lpf_out_;
}

template <typename T>
void DigitalLpFilter<T>::clear()
{
  Lpf_in_prev_[1] = 0;
  Lpf_in_prev_[0] = 0;
  Lpf_out_prev_[1] = 0;
  Lpf_out_prev_[0] = 0;
}

template class DigitalLpFilter<double>;
template class DigitalLpFilter<float>;

/*============================================================================*/

template <typename T>
DerivLpFilter<T>::DerivLpFilter(T w_c, T t_s)
{
  Lpf_in_prev_[0] = 0;
  Lpf_in_prev_[1] = 0;
  Lpf_out_prev_[0] = 0;
  Lpf_out_prev_[1] = 0;
  Lpf_in1_ = 0;
  Lpf_in2_ = 0;
  Lpf_in3_ = 0;
  Lpf_out1_ = 0;
  Lpf_out2_ = 0;
  T a = 1.4142;
  T den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;

  Lpf_in1_ = 2 * t_s * w_c * w_c / den;
  Lpf_in2_ = 0;
  Lpf_in3_ = -2. * t_s * w_c * w_c / den;
  Lpf_out1_ = -1. * (-8 + t_s * t_s * w_c * w_c * 2) / den;
  Lpf_out2_ = -1. * (4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c) / den;
  lpf_out_ = 0.0;
  clear();
}

template <typename T>
DerivLpFilter<T>::~DerivLpFilter() = default;

template <typename T>
void DerivLpFilter<T>::input(T lpf_in)
{
  // static int i(0);
  lpf_out_ = Lpf_in1_ * lpf_in + Lpf_in2_ * Lpf_in_prev_[0] + Lpf_in3_ * Lpf_in_prev_[1] +  // input component
             Lpf_out1_ * Lpf_out_prev_[0] + Lpf_out2_ * Lpf_out_prev_[1];                   // output component

  Lpf_in_prev_[1] = Lpf_in_prev_[0];
  Lpf_in_prev_[0] = lpf_in;
  Lpf_out_prev_[1] = Lpf_out_prev_[0];
  Lpf_out_prev_[0] = lpf_out_;
  // ++i;
}

template <typename T>
T DerivLpFilter<T>::output()
{
  return lpf_out_;
}

template <typename T>
void DerivLpFilter<T>::clear()
{
  Lpf_in_prev_[1] = 0;
  Lpf_in_prev_[0] = 0;
  Lpf_out_prev_[1] = 0;
  Lpf_out_prev_[0] = 0;
}

template class DerivLpFilter<double>;
template class DerivLpFilter<float>;

/*============================================================================*/

template <typename T>
FF01Filter<T>::FF01Filter(float t_s, float w_c)
{
  Lpf_in_prev_[0] = Lpf_in_prev_[1] = 0;
  Lpf_out_prev_[0] = Lpf_out_prev_[1] = 0;
  Lpf_in1_ = 0, Lpf_in2_ = 0, Lpf_in3_ = 0, Lpf_out1_ = 0, Lpf_out2_ = 0;
  T a = 1.4142;
  T den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;
  T J = 0.00008;
  T B = 0.0002;

  Lpf_in1_ = B * t_s * t_s * w_c * w_c + 2 * J * t_s * w_c * w_c;
  Lpf_in2_ = 2 * B * t_s * t_s * w_c * w_c;
  Lpf_in3_ = B * t_s * t_s * w_c * w_c - 2 * J * t_s * w_c * w_c;
  Lpf_out1_ = -1. * (-8 + t_s * t_s * w_c * w_c * 2) / den;
  Lpf_out2_ = -1. * (4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c) / den;
}

template <typename T>
FF01Filter<T>::~FF01Filter() = default;

template <typename T>
void FF01Filter<T>::input(T lpf_in)
{
  lpf_out_ = Lpf_in1_ * lpf_in + Lpf_in2_ * Lpf_in_prev_[0] + Lpf_in3_ * Lpf_in_prev_[1] +  // input component
             Lpf_out1_ * Lpf_out_prev_[0] + Lpf_out2_ * Lpf_out_prev_[1];                   // output component
  Lpf_in_prev_[1] = Lpf_in_prev_[0];
  Lpf_in_prev_[0] = lpf_in;
  Lpf_out_prev_[1] = Lpf_out_prev_[0];
  Lpf_out_prev_[0] = lpf_out_;
}

template <typename T>
T FF01Filter<T>::output()
{
  return lpf_out_;
}

template <typename T>
void FF01Filter<T>::clear()
{
  Lpf_in_prev_[1] = 0;
  Lpf_in_prev_[0] = 0;
  Lpf_out_prev_[1] = 0;
  Lpf_out_prev_[0] = 0;
}

template class FF01Filter<float>;
template class FF01Filter<double>;

/*============================================================================*/

template <typename T>
FF02Filter<T>::FF02Filter(float t_s, float w_c)
{
  T J = 0.003216;

  Lpf_in_prev_[0] = Lpf_in_prev_[1] = 0;
  Lpf_out_prev_[0] = Lpf_out_prev_[1] = 0;
  Lpf_in1_ = 0, Lpf_in2_ = 0, Lpf_in3_ = 0, Lpf_out1_ = 0, Lpf_out2_ = 0;

  T a = 1.4142;
  T den = 4 + 2 * a * w_c * t_s + t_s * t_s * w_c * w_c;

  Lpf_in1_ = J * 2 * t_s * w_c * w_c / den;
  Lpf_in2_ = 0;
  Lpf_in3_ = -2. * J * t_s * w_c * w_c / den;
  Lpf_out1_ = -1. * (-8 + t_s * t_s * w_c * w_c * 2) / den;
  Lpf_out2_ = -1. * (4 - 2 * a * w_c * t_s + t_s * t_s * w_c * w_c) / den;

  clear();
}

template <typename T>
FF02Filter<T>::~FF02Filter() = default;

template <typename T>
void FF02Filter<T>::input(T lpf_in)
{
  lpf_out_ = Lpf_in1_ * lpf_in + Lpf_in2_ * Lpf_in_prev_[0] + Lpf_in3_ * Lpf_in_prev_[1] +  // input component
             Lpf_out1_ * Lpf_out_prev_[0] + Lpf_out2_ * Lpf_out_prev_[1];                   // output component
  Lpf_in_prev_[0] = lpf_in;
  Lpf_in_prev_[1] = Lpf_in_prev_[0];
  Lpf_out_prev_[0] = lpf_out_;
  Lpf_out_prev_[1] = Lpf_out_prev_[0];
}

template <typename T>
T FF02Filter<T>::output()
{
  return lpf_out_;
}

template <typename T>
void FF02Filter<T>::clear()
{
  Lpf_in_prev_[1] = 0;
  Lpf_in_prev_[0] = 0;
  Lpf_out_prev_[1] = 0;
  Lpf_out_prev_[0] = 0;
}

template class FF02Filter<float>;
template class FF02Filter<double>;

/*============================================================================*/

template <typename T>
AverageFilter<T>::AverageFilter(T dt, T t_const, T limit) : dt_(dt), t_const_(t_const), limit_(limit)
{
  est_value_ = 0.;
}

template <typename T>
AverageFilter<T>::~AverageFilter()
{
  est_value_ = 0;
}

template <typename T>
void AverageFilter<T>::clear()
{
  est_value_ = 0.;
}

template <typename T>
void AverageFilter<T>::input(T input)
{
  T update_value = input - est_value_;
  if (fabs(update_value) > limit_)
  {
    update_value = 0.;
  }
  est_value_ += (dt_ / (dt_ + t_const_)) * update_value;
}

template <typename T>
T AverageFilter<T>::output()
{
  return est_value_;
}

template class AverageFilter<float>;
template class AverageFilter<double>;

/*============================================================================*/

template <typename T>
RampFilter<T>::RampFilter(T acc, T dt)
{
  acc_ = acc;
  dt_ = dt;
  RampFilter::clear();
}

template <typename T>
void RampFilter<T>::input(T input_value)
{
  last_value_ += minAbs(input_value - last_value_, acc_ * dt_);
}

template <typename T>
void RampFilter<T>::clear()
{
  last_value_ = 0.;
}

template <typename T>
void RampFilter<T>::clear(T last_value)
{
  last_value_ = last_value;
}

template <typename T>
void RampFilter<T>::setAcc(T acc)
{
  acc_ = acc;
}

template <typename T>
T RampFilter<T>::output()
{
  return last_value_;
}

template class RampFilter<float>;
template class RampFilter<double>;

/*============================================================================*/

template <typename T>
OneEuroFilter<T>::OneEuroFilter(double _freq, T _mincutoff, T _beta, T _dcutoff)
  : freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff)
{
  firsttime = true;
  x_prev = 0;
  hatxprev = 0;
  dhatxprev = 0;
  filtered_val = 0;
};

template <typename T>
OneEuroFilter<T>::~OneEuroFilter() = default;

template <typename T>
void OneEuroFilter<T>::input(T input_value)
{
  T dx = 0;
  if (firsttime)
    dhatxprev = dx;
  else
    dx = (input_value - x_prev) * freq;
  T edx = alpha(dcutoff, freq) * dx + (1 - alpha(dcutoff, freq)) * dhatxprev;
  dhatxprev = edx;
  T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));

  if (firsttime)
    hatxprev = input_value;
  filtered_val = alpha(cutoff, freq) * input_value + (1 - alpha(cutoff, freq)) * hatxprev;
  hatxprev = filtered_val;
  firsttime = false;
}

template <typename T>
T OneEuroFilter<T>::output()
{
  return filtered_val;
}

template <typename T>
void OneEuroFilter<T>::clear()
{
  firsttime = true;
  x_prev = 0;
  hatxprev = 0;
  dhatxprev = 0;
}

template class OneEuroFilter<float>;
template class OneEuroFilter<double>;
