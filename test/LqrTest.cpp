//
// Created by chenzheng on 3/20/21.
//

#include "rm_common/lqr.h"
#include <gtest/gtest.h>

TEST(Lqr, lqr
) {
  static const size_t STATE_DIM = 6;
  static const size_t CONTROL_DIM = 2;

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> A{}, Q{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R{};
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> K{};

  A << 0, 1, 0, 0, 0, 0,
      432.8077, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      73.9215, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0;

  B << 0, 0,
      -49.6930, -49.6930,
      0, 0,
      7.3620, 7.3620,
      0, 0,
      -26.3737, 26.3737;

  Q << 10, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 10, 0, 0, 0,
      0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;

  R << 1, 0,
      0, 1;

  K << -36.4112, -1.51982, -2.23607, -2.45734, -0.707107, -0.725818,
      -36.4112, -1.51982, -2.23607, -2.45734, 0.707107, 0.725818;

  Lqr<double> lqr(A, B, Q, R);
  lqr.computeK();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> average = lqr.getK();
  for (uint j = 0; j < CONTROL_DIM; ++j) {
    for (uint i = 0; i < STATE_DIM; ++i) {
      EXPECT_NEAR(K(j, i), average(j, i), 1e-3);
    }
  }
}