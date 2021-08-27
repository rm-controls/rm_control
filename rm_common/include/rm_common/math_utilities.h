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
// Created by qiayuan on 12/22/19.
//

#pragma once

#include <cmath>
template <typename T>
T angularMinus(T a, T b)
{
  a = fmod(a, 2.0 * M_PI);
  b = fmod(b, 2.0 * M_PI);

  T res1 = a - b;
  T res2 = (a < b) ? (a + 2 * M_PI - b) : (a - 2 * M_PI - b);

  return (std::abs(res1) < std::abs(res2)) ? res1 : res2;
}

template <typename T>
T minAbs(T a, T b)
{
  T sign = (a < 0.0) ? -1.0 : 1.0;
  return sign * fmin(fabs(a), b);
}

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

template <typename T>
T square(T val)
{
  return val * val;
}

template <typename T>
T alpha(T cutoff, double freq)
{
  T tau = 1.0 / (2 * M_PI * cutoff);
  T te = 1.0 / freq;
  return 1.0 / (1.0 + tau / te);
}
