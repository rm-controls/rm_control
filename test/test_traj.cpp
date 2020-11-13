//
// Created by qiayuan on 3/21/20.
//

#include "traj_gen.h"
#include <ros/ros.h>
#include <rm_msgs/Joint.h>

int main(int argc, char **argv) {
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
  else {
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

    while (ros::ok() && !traj.isReach(t)) {
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
  while (ros::ok() && !min_traj.isReach()) {
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
