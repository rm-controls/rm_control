//
// Created by qiayuan on 8/15/20.
//

#include <iostream>
#include "bullet_solver.h"

int main(int argc, char **argv) {
  Iter2DSolver<double> iter2d(0.1, 9.8, 0.01, 0.0001, 3.);
  Approx2DSolver<double> approx2d(0.1, 9.8, 0.01, 0.01, 3.);
  Iter3DSolver<double> iter3d(0.1, 9.8, 0.01, 0.0001, 3.);
  Approx3DSolver<double> approx3d(0.1, 9.8, 0.01, 0.0001, 3.);

  double bullet_speed = 18.;
  double angle_init[2]{}, angle_solved[2]{};

  double pos_2d[2] = {10, 1};
  double vel_2d[2] = {1, 1};

  iter2d.setTarget(pos_2d, vel_2d);
  iter2d.setBulletSpeed(bullet_speed);
  iter2d.solve(angle_init);
  iter2d.output(angle_solved);
  std::cout << angle_solved[0] << std::endl;
  approx2d.setTarget(pos_2d, vel_2d);
  approx2d.setBulletSpeed(bullet_speed);
  approx2d.solve(angle_init);
  approx2d.output(angle_solved);
  std::cout << angle_solved[0] << std::endl;

  double pos_3d[3] = {7, 0, 1};
  double vel_3d[3] = {0, 1, 0};

  iter3d.setTarget(pos_3d, vel_3d);
  iter3d.setBulletSpeed(bullet_speed);
  iter3d.solve(angle_init);
  iter3d.output(angle_solved);
  std::cout << "yaw:" << angle_solved[0] << " pitch:" << angle_solved[1]
            << std::endl;
  approx3d.setTarget(pos_3d, vel_3d);
  approx3d.setBulletSpeed(bullet_speed);
  approx3d.solve(angle_init);
  approx3d.output(angle_solved);
  std::cout << "yaw:" << angle_solved[0] << " pitch:" << angle_solved[1]
            << std::endl;

  return 0;
}
