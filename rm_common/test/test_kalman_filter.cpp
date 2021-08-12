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
// Created by qiayuan on 4/3/20.
//
#include "rm_common/filters/kalman_filter.h"
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define SAMPLE_RATE 1000
#define ACTUAL_RATE 10
#define NOISE_MAG 1.

int main(int argc, char** argv)
{
  // Set up filter
  Mat2<double> A, B, H, Q, R;
  Vec2<double> x, u;

  A << 1., 1. / SAMPLE_RATE, 0., 1.;

  B << 0., 0., 0, 0;

  H << 1., 0., 0., 1.;

  Q << 300., 0., 0., 300.;

  R << 2000., 0., 0., 2000.;

  x << 0., 0.;
  u << 0., 0.;
  KalmanFilter<double> filter(A, B, H, Q, R);
  filter.clear(x);

  // Set up normal distribution
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, NOISE_MAG);

  // Set up ros
  ros::init(argc, argv, "traj_test");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<geometry_msgs::Twist>("test_cmd", 100);
  geometry_msgs::Twist data{};
  ros::Rate loop_rate(SAMPLE_RATE);

  // Test
  int i = 0;
  double x_last = 0.;
  double dis = 0.;
  double t = 0.;
  Vec2<double> x_hat{};
  while (ros::ok() && t < 2.)
  {
    if (i >= SAMPLE_RATE / ACTUAL_RATE)
    {  // oversampling
      i = 0;
      x[0] = 20. * sin(2. * M_PI * t) + distribution(generator);
      x[1] = (x[0] - x_last) * (double)ACTUAL_RATE;
      x_last = x[0];
      filter.predict(u);
      filter.update(x);
    }
    else
    {
      filter.predict(u);
    }
    x_hat = filter.getState();
    data.linear.x = 20. * sin(2. * M_PI * t);
    data.linear.y = x[0];
    data.linear.z = x_hat[0];
    dis += pow(data.linear.z - data.linear.x, 2);
    // dis += pow(x[1], 2);
    pub.publish(data);
    ++i;
    t += 1. / (double)SAMPLE_RATE;
    loop_rate.sleep();
  }

  std::cout << sqrt(dis) << std::endl;
  return 0;
}
