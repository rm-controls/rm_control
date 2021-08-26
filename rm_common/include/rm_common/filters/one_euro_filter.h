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
// Created by liucong on 2020/12/5.
//

#pragma once

#include <cmath>

template <typename T>
class OneEuroFilter
{
public:
  OneEuroFilter(double _freq, T _mincutoff, T _beta, T _dcutoff)
    : freq(_freq), mincutoff(_mincutoff), beta(_beta), dcutoff(_dcutoff)
  {
    firsttime = true;
    x_prev = 0;
    hatxprev = 0;
    dhatxprev = 0;
    filtered_val = 0;
  };

  ~OneEuroFilter() = default;

  void input(T input_value)
  {
    T dx = 0;
    if (!firsttime)
      dx = (input_value - x_prev) * freq;
    if (firsttime)
      dhatxprev = dx;
    T edx = alpha(dcutoff, freq) * dx + (1 - alpha(dcutoff, freq)) * dhatxprev;
    dhatxprev = edx;
    T cutoff = mincutoff + beta * std::abs(static_cast<double>(edx));

    if (firsttime)
      hatxprev = input_value;
    filtered_val = alpha(cutoff, freq) * input_value + (1 - alpha(cutoff, freq)) * hatxprev;
    hatxprev = filtered_val;
    firsttime = false;
  };

  T alpha(T cutoff, double freq)
  {
    T tau = 1.0 / (2 * M_PI * cutoff);
    T te = 1.0 / freq;
    return 1.0 / (1.0 + tau / te);
  }

  T output()
  {
    return filtered_val;
  };

  void clear()
  {
    firsttime = true;
    x_prev = 0;
    hatxprev = 0;
    dhatxprev = 0;
  };

private:
  double freq;
  bool firsttime;
  T mincutoff, beta, dcutoff;
  T x_prev, dhatxprev, hatxprev;
  T filtered_val;
};
