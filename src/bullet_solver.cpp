
//
// Created by qiayuan on 8/14/20.
//

#include "bullet_solver.h"
#include <cmath>
///////////////////////////BulletSolver/////////////////////////////
template
class BulletSolver<double>;
template
class BulletSolver<float>;

///////////////////////////Bullet2DSolver/////////////////////////////
template<typename T>
void Bullet2DSolver<T>::solve(const T *angle_init) {
  T pitch_point = std::atan2(target_z_, target_x_);
  T error_point = computeError(pitch_point);
  T error_init = computeError(angle_init[0]);
  //compare pitch angle which direct pointing to target and angle provide by user
  pitch_solved_ = error_init > error_point ? pitch_point : angle_init[0];
  T error_z = error_init > error_point ? error_point : error_init;
  T temp_z = this->target_x_ * std::tan(pitch_solved_);

  int count = 0;
  while (std::abs(error_z) >= 0.000001) {
    temp_z = temp_z + error_z;
    pitch_solved_ = std::atan2(temp_z, this->target_x_);
    error_z = computeError(pitch_solved_);
    if (count >= 20 || error_z == 999999.) {
      pitch_solved_ = angle_init[0];
      break;
    }
    count++;
  }
}

template
class Bullet2DSolver<double>;
template
class Bullet2DSolver<float>;

///////////////////////////Iter2DSolver/////////////////////////////
template<typename T>
T Iter2DSolver<T>::computeError(T pitch) {
  T rt_target_x = this->target_x_;
  T rt_target_z = this->target_z_;
  T rt_bullet_x{}, rt_bullet_z{};
  T bullet_v_x = this->bullet_speed_ * std::cos(pitch);
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_x <= rt_target_x) {
    this->fly_time_ += this->dt_;
    rt_bullet_x = (1 / this->resistance_coff_) * bullet_v_x
        * (1 - std::exp(-this->fly_time_ * this->resistance_coff_));
    rt_target_x += this->target_dx_ * this->dt_;

    //avoid keep looping cause by null solution
    if (this->fly_time_ > this->timeout_) {
      return 999999.;
    }
  }
  rt_bullet_z = (1 / this->resistance_coff_)
      * (bullet_v_z + this->g_ / this->resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * this->resistance_coff_))
      - this->fly_time_ * this->g_ / this->resistance_coff_;
  rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  return rt_target_z - rt_bullet_z;
}

template
class Iter2DSolver<double>;
template
class Iter2DSolver<float>;

///////////////////////////Approx2DSolver/////////////////////////////
template<typename T>
T Approx2DSolver<T>::computeError(T pitch) {

  this->fly_time_ = (-log(1 - this->target_x_ * this->resistance_coff_
      / (this->bullet_speed_ * cos(pitch) - this->target_dx_)))
      / this->resistance_coff_;
  if (std::isnan(this->fly_time_))
    return 999999.;
  T rt_bullet_z =
      ((this->bullet_speed_ * sin(pitch) - this->target_dz_)
          + (this->g_ / this->resistance_coff_))
          * (1 - std::exp(-this->resistance_coff_ * this->fly_time_))
          / this->resistance_coff_ - this->g_ * this->fly_time_
          / this->resistance_coff_;

  return this->target_z_ - rt_bullet_z;
}

template
class Approx2DSolver<double>;
template
class Approx2DSolver<float>;

///////////////////////////Bullet3DSolver/////////////////////////////
template<typename T>
void Bullet3DSolver<T>::solve(const T *angle_init) {
  T error_theta_z_init[2]{}, error_theta_z_point[2]{};

  T error_init = computeError(angle_init[0], angle_init[1], error_theta_z_init);

  T yaw_point = std::atan2(target_y_, target_x_);
  T pitch_point = (T) std::atan2(
      target_z_, std::sqrt(std::pow(target_x_, 2) + std::pow(target_y_, 2)));
  T error_point = computeError(yaw_point, pitch_point, error_theta_z_point);

  //compare pitch and yaw angle which direct pointing to target and angle provide by user
  if (error_init > error_point) {
    yaw_solved_ = yaw_point;
    pitch_solved_ = pitch_point;
  } else {
    yaw_solved_ = angle_init[0];
    pitch_solved_ = angle_init[1];
  }

  T error_theta_z[2] =
      {error_init > error_point ? error_theta_z_init[0]
                                : error_theta_z_point[0],
       error_init > error_point ? error_theta_z_init[1]
                                : error_theta_z_point[1]};
  T temp_z = target_x_ / cos(yaw_solved_) * tan(pitch_solved_);

  int count = 0;
  T error = 999999;
  while (error >= 0.00001) {
    error = computeError(yaw_solved_, pitch_solved_, error_theta_z);
    yaw_solved_ = yaw_solved_ + error_theta_z[0];
    temp_z = temp_z + error_theta_z[1];
    pitch_solved_ = std::atan2(temp_z, std::sqrt(std::pow(target_x_, 2)
                                                     + std::pow(target_y_, 2)));
    if (count >= 20) {
      yaw_solved_ = angle_init[0];
      pitch_solved_ = angle_init[1];
      break;
    }
    count++;
  }
}

template
class Bullet3DSolver<double>;
template
class Bullet3DSolver<float>;

///////////////////////////Iter3DSolver/////////////////////////////
template<typename T>
double Iter3DSolver<T>::computeError(T yaw, T pitch, T *error) {
  T rt_target_x = this->target_x_;
  T rt_target_y = this->target_y_;
  T rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));
  T rt_bullet_rho{}, rt_bullet_z{};

  T bullet_v_rho = this->bullet_speed_ * std::cos(pitch);
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch);

  this->fly_time_ = 0;
  while (rt_bullet_rho <= rt_target_rho) {
    this->fly_time_ += this->dt_;

    rt_bullet_rho = (1 / this->resistance_coff_) * bullet_v_rho
        * (1 - std::exp(-this->fly_time_ * this->resistance_coff_));

    rt_target_x += this->target_dx_ * this->dt_;
    rt_target_y += this->target_dy_ * this->dt_;
    rt_target_rho =
        std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

    //avoid keep looping cause by null solution
    if (this->fly_time_ > this->timeout_)
      return 999999.;

  }

  T rt_target_theta = std::atan2(rt_target_y, rt_target_x);
  T rt_target_z = this->target_z_ + this->target_dz_ * this->fly_time_;

  T rt_bullet_theta = yaw;
  rt_bullet_z = (1 / this->resistance_coff_)
      * (bullet_v_z + this->g_ / this->resistance_coff_)
      * (1 - std::exp(-this->fly_time_ * this->resistance_coff_))
      - this->fly_time_ * this->g_ / this->resistance_coff_;

  error[0] = rt_target_theta - rt_bullet_theta;
  error[1] = rt_target_z - rt_bullet_z;

  return std::sqrt(
      std::pow(error[0] * rt_bullet_rho, 2) + std::pow(error[1], 2));
}

template
class Iter3DSolver<double>;
template
class Iter3DSolver<float>;

template<typename T>
double Approx3DSolver<T>::computeError(T yaw, T pitch, T *error) {
  T rt_target_x = this->target_x_;
  T rt_target_y = this->target_y_;
  T rt_target_rho =
      std::sqrt(std::pow(rt_target_x, 2) + std::pow(rt_target_y, 2));

  T target_v_rho =
      std::cos(yaw) * this->target_dx_ + std::sin(yaw) * this->target_dy_;
  T bullet_v_rho = this->bullet_speed_ * std::cos(pitch) - target_v_rho;
  T bullet_v_z = this->bullet_speed_ * std::sin(pitch) - this->target_dz_;

  this->fly_time_ = (-std::log(1 - rt_target_rho * this->resistance_coff_
      / bullet_v_rho)) / this->resistance_coff_;
  if (std::isnan(this->fly_time_))
    return 999999.;
  T rt_bullet_z =
      (bullet_v_z + (this->g_ / this->resistance_coff_))
          * (1 - std::exp(-this->resistance_coff_ * this->fly_time_))
          / this->resistance_coff_ - this->g_ * this->fly_time_
          / this->resistance_coff_;

  T rt_target_theta =
      std::atan2(this->target_y_ + this->target_dy_ * this->fly_time_,
                 this->target_x_ + this->target_dx_ * this->fly_time_);

  error[0] = rt_target_theta - yaw;
  error[1] = this->target_z_ - rt_bullet_z;
  return std::sqrt(
      std::pow(error[0] * rt_target_rho, 2) + std::pow(error[1], 2));
}

template
class Approx3DSolver<double>;
template
class Approx3DSolver<float>;
