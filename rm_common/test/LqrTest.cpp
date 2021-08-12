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

#include "rm_common/lqr.h"
#include <gtest/gtest.h>

TEST(Lqr, lqr)
{
  static const size_t STATE_DIM = 6;
  static const size_t CONTROL_DIM = 2;

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> A{}, Q{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R{};
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> K{};
  Eigen::Matrix<double, CONTROL_DIM, 1> Nbar{};

  A << 0, 1, 0, 0, 0, 0, 432.8077, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 73.9215, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0;

  B << 0, 0, -49.6930, -49.6930, 0, 0, 7.3620, 7.3620, 0, 0, -26.3737, 26.3737;

  Q << 10, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  R << 1, 0, 0, 1;

  K << -36.4111581829, -1.5198212494, -2.2360679774, -2.4573417169, -0.7071067811, -0.7258175096, -36.4111581829,
      -1.5198212494, -2.2360679774, -2.4573417169, 0.7071067811, 0.7258175096;

  Lqr<double> lqr(A, B, Q, R);
  lqr.computeK();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> average = lqr.getK();
  for (uint j = 0; j < CONTROL_DIM; ++j)
  {
    for (uint i = 0; i < STATE_DIM; ++i)
    {
      EXPECT_NEAR(K(j, i), average(j, i), 1e-9);
    }
  }
}
