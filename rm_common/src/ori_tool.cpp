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
// Created by qiayuan on 8/13/20.
//

#include "rm_common/ori_tool.h"
#include "rm_common/math_utilities.h"
#include <eigen3/Eigen/Eigenvalues>

void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), square(q.w) + square(q.x) - square(q.y) - square(q.z));
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), square(q.w) - square(q.x) - square(q.y) + square(q.z));
}

double yawFromQuat(const geometry_msgs::Quaternion& q)
{
  double roll, pitch, yaw;
  quatToRPY(q, roll, pitch, yaw);
  return yaw;
}

tf::Quaternion getAverageQuaternion(const std::vector<tf::Quaternion>& quaternions, const std::vector<double>& weights)
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
  Eigen::Vector3d vec;
  for (size_t i = 0; i < quaternions.size(); ++i)
  {
    // Weigh the quaternions according to their associated weight
    tf::Quaternion quat = quaternions[i] * weights[i];
    // Append the weighted Quaternion to a matrix Q.
    Q(0, i) = quat.x();
    Q(1, i) = quat.y();
    Q(2, i) = quat.z();
    Q(3, i) = quat.w();
  }
  // Creat a solver for finding the eigenvectors and eigenvalues
  Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());
  // Find index of maximum (real) Eigenvalue.
  auto eigenvalues = es.eigenvalues();
  size_t max_idx = 0;
  double max_value = eigenvalues[max_idx].real();
  for (size_t i = 1; i < 4; ++i)
  {
    double real = eigenvalues[i].real();
    if (real > max_value)
    {
      max_value = real;
      max_idx = i;
    }
  }
  // Get corresponding Eigenvector, normalize it and return it as the average quat
  auto eigenvector = es.eigenvectors().col(max_idx).normalized();
  tf::Quaternion mean_orientation(eigenvector[0].real(), eigenvector[1].real(), eigenvector[2].real(),
                                  eigenvector[3].real());
  return mean_orientation;
}

tf::Quaternion rotationMatrixToQuaternion(const Eigen::Map<Eigen::Matrix3d>& rot)
{
  Eigen::Matrix3d r = rot.transpose();
  tf::Quaternion quat;
  double trace = r.trace();
  if (trace > 0.0)
  {
    double s = sqrt(trace + 1.0) * 2.0;
    quat.setValue((r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s, 0.25 * s);
  }
  else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2)))
  {
    double s = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    quat.setValue(0.25 * s, (r(0, 1) + r(1, 0)) / s, (r(0, 2) + r(2, 0)) / s, (r(2, 1) - r(1, 2)) / s);
  }
  else if (r(1, 1) > r(2, 2))
  {
    double s = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    quat.setValue((r(0, 1) + r(1, 0)) / s, (r(0, 1) + r(1, 0)) / s, 0.25 * s, (r(0, 2) - r(2, 0)) / s);
  }
  else
  {
    double s = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    quat.setValue((r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s, (r(1, 0) - r(0, 1)) / s);
  }
  return quat;
}
