//
// Created by astro on 2021/5/15.
//
#ifndef RM_COMMON_CONTROLLER_MANAGER_H_
#define RM_COMMON_CONTROLLER_MANAGER_H_
#include "rm_common/decision/service_caller.h"

#include <algorithm>

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>

namespace rm_common {
class ControllerManager {
 public:
  explicit ControllerManager(ros::NodeHandle &nh) :
      load_client_(nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller")),
      switch_caller_(nh) {
    if (!nh.hasParam("controllers_list"))
      ROS_ERROR("No controllers defined");
    ROS_INFO("Waiting for load_controller service...");
    load_client_.waitForExistence();
    ros::NodeHandle nh_list(nh, "controllers_list");
    XmlRpc::XmlRpcValue controllers;
    if (!nh.getParam("state_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i) {
        state_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
    if (!nh.getParam("main_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i) {
        main_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
    if (!nh.getParam("calibration_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i) {
        calibration_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
  }
  void update() {
    if (!switch_caller_.isCalling() && switch_caller_.getOk()) {
      if (!start_buffer_.empty())
        switch_caller_.startControllers(start_buffer_);
      if (!stop_buffer_.empty())
        switch_caller_.stopControllers(stop_buffer_);
      if (!stop_buffer_.empty() || !stop_buffer_.empty()) {
        switch_caller_.callService();
        start_buffer_.clear();
        stop_buffer_.clear();
      }
    }
  }

 private:
  void loadController(const std::string &controller) {
    controller_manager_msgs::LoadController load_controller;
    load_controller.request.name = controller;
    load_client_.call(load_controller);
    if (load_controller.response.ok)
      ROS_INFO("Loaded %s", controller.c_str());
    else
      ROS_ERROR("Fail to load %s", controller.c_str());
  }
  ros::ServiceClient load_client_;
  std::vector<std::string> state_controllers_, main_controllers_, calibration_controllers_;
  std::vector<std::string> start_buffer_, stop_buffer_;
  SwitchControllersServiceCaller switch_caller_;
};

}
#endif //RM_COMMON_CONTROLLER_MANAGER_H_
