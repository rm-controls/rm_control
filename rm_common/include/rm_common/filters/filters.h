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

#pragma once

template <typename T>
class Filter
{
public:
  Filter() = default;
  virtual ~Filter() = default;
  virtual void input(T input_value) = 0;
  virtual T output() = 0;
  virtual void clear() = 0;
};

template <typename T>
class ButterworthFilter : public Filter<T>
{
public:
  ButterworthFilter(int num_sample, T dt, T cutoff_frequency);
  ~ButterworthFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  T* mpBuffer_;
  int mCurIdx_;
  int mNumSample_;
  T mDt_;
  T mCutoffFreq_;
  T mValue_;
};

template <typename T>
class DigitalLpFilter : public Filter<T>
{
public:
  DigitalLpFilter(T w_c, T t_s);
  ~DigitalLpFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  T Lpf_in_prev_[2];
  T Lpf_out_prev_[2];
  T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
  T lpf_out_;
};

template <typename T>
class MovingAverageFilter : public Filter<T>
{
public:
  explicit MovingAverageFilter(int num_data);
  ~MovingAverageFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  T* buffer_;
  int num_data_;
  int idx_;
  T sum_;
};

template <typename T>
class DerivLpFilter : public Filter<T>
{
public:
  DerivLpFilter(T w_c, T t_s);
  ~DerivLpFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  T Lpf_in_prev_[2];
  T Lpf_out_prev_[2];
  T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
  T lpf_out_;
};

template <typename T>
class FF01Filter : public Filter<T>
{
public:
  FF01Filter(float t_s, float w_c);
  virtual ~FF01Filter();
  virtual void input(T input_value);
  virtual T output();
  virtual void clear();

private:
  T Lpf_in_prev_[2];
  T Lpf_out_prev_[2];
  T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
  T lpf_out_;
};

template <typename T>
class FF02Filter : public Filter<T>
{
public:
  FF02Filter(float t_s, float w_c);
  ~FF02Filter();
  void input(T input_value);
  T output();
  void clear();

private:
  T Lpf_in_prev_[2];
  T Lpf_out_prev_[2];
  T Lpf_in1_, Lpf_in2_, Lpf_in3_, Lpf_out1_, Lpf_out2_;
  T lpf_out_;
};

template <typename T>
class AverageFilter : public Filter<T>
{
public:
  AverageFilter(T dt, T t_const, T limit);
  ~AverageFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  T est_value_;
  T dt_;
  T t_const_;
  T limit_;
};

template <typename T>
class RampFilter : public Filter<T>
{
public:
  RampFilter(T acc, T dt);
  ~RampFilter() = default;
  void input(T input_value);
  void clear();
  void clear(T last_value);
  void setAcc(T acc);  // without clear.
  T output();

private:
  T last_value_;
  T acc_;
  T dt_;
};

template <typename T>
class OneEuroFilter : public Filter<T>
{
public:
  OneEuroFilter(double _freq, T _mincutoff, T _beta, T _dcutoff);
  ~OneEuroFilter();
  void input(T input_value);
  T output();
  void clear();

private:
  double freq;
  bool firsttime;
  T mincutoff, beta, dcutoff;
  T x_prev, dhatxprev, hatxprev;
  T filtered_val;
};
