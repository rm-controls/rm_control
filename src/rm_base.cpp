//
// Created by qiayuan on 12/27/20.
//

#include "rm_base/control_loop.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rm_base");
  ros::NodeHandle nh;
  ros::NodeHandle robot_hw_nh("~");


  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(2);
  spinner.start();

  struct sched_param params{.sched_priority = 98};
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    ROS_ERROR("Set scheduler failed, RUN THIS NODE AS SUPER USER.\n");

  try {
    // Create the hardware interface specific to your robot
    std::shared_ptr<rm_base::RmBaseHardWareInterface>
        rm_base_hw_interface = std::make_shared<rm_base::RmBaseHardWareInterface>();
    // Initialise the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    rm_base_hw_interface->init(nh, robot_hw_nh);

    // Start the control loop
    rm_base::RmBaseLoop control_loop(nh, rm_base_hw_interface);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception &e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n" << "\t" << e.what());
    return 1;
  }

  return 0;
}

