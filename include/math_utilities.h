//
// Created by qiayuan on 12/22/19.
//

#ifndef SRC_RM_COMMON_INCLUDE_MATH_UTILITIES_H_
#define SRC_RM_COMMON_INCLUDE_MATH_UTILITIES_H_

#include <cmath>
template<typename T>
T angularMinus(T a, T b) {
  a = fmod(a, 2.0 * M_PI);
  b = fmod(b, 2.0 * M_PI);

  T res1 = a - b;
  T res2 = (a < b) ? (a + 2 * M_PI - b) : (a - 2 * M_PI - b);

  return (std::abs(res1) < std::abs(res2)) ? res1 : res2;
}

template<typename T>
T minAbs(T a, T b) {
  T sign = (a < 0.0) ? -1.0 : 1.0;
  return sign * fmin(fabs(a), b);
}

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template<typename T>
T square(T val) {
  return val * val;
}

#endif //SRC_RM_COMMON_INCLUDE_MATH_UTILITIES_H_
