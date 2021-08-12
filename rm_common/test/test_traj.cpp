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
// Created by qiayuan on 3/21/20.
//

#include "rm_common/traj_gen.h"
#include <ros/ros.h>
//#include <rm_msgs/Joint.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_test");

  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<rm_msgs::Joint>("test_cmd", 100);
  rm_msgs::Joint cmd{};

  RampTraj<double> traj;
  traj.setLimit(1.);
  traj.setState(0., 1., 0.);
  ros::Rate loop_rate(100);

  if (!traj.calc(2.5))
    ROS_ERROR("Acc too small");
  else
  {
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();
    pub.publish(cmd);
    loop_rate.sleep();

    double t = -1.;

    while (ros::ok() && !traj.isReach(t))
    {
      cmd.q_des[0] = traj.getPos(t);
      cmd.qd_des[0] = traj.getVel(t);
      cmd.ff[0] = traj.getAcc(t);
      t += 0.01;
      pub.publish(cmd);

      loop_rate.sleep();
    }
  }
  cmd.q_des[0] = 0.;
  cmd.qd_des[0] = 0.;
  cmd.ff[0] = 0.;

  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  pub.publish(cmd);
  loop_rate.sleep();
  MinTimeTraj<double> min_traj;
  min_traj.setLimit(1., 1., 0.01);
  min_traj.setTarget(1.);
  double s[3]{};
  while (ros::ok() && !min_traj.isReach())
  {
    s[2] = min_traj.getTau(s[0], s[1]) / 1.;
    s[1] += 0.01 * s[2];
    s[0] += 0.01 * s[1];

    cmd.q_des[0] = s[0];
    cmd.qd_des[0] = s[1];
    cmd.ff[0] = s[2];
    pub.publish(cmd);
    loop_rate.sleep();
  }
  return 0;
}
