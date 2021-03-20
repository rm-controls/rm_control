//
// Created by chenzheng on 3/20/21.
//

#include "rm_common/lqr.h"

int main(int argc, char **argv) {
  static const size_t STATE_DIM = 6;
  static const size_t CONTROL_DIM = 2;

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> A{}, Q{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> R{};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K{};

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

  Lqr<double> lqr(A, B, Q, R);
  lqr.computeK();
  K = lqr.getK();
  std::cout << K << std::endl;

  return 0;
}