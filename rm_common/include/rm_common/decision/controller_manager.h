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
// Created by astro on 2021/5/15.
//
#pragma once

#include "rm_common/decision/service_caller.h"

#include <algorithm>

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>

namespace rm_common
{
class ControllerManager
{
public:
  explicit ControllerManager(ros::NodeHandle& nh)
    : load_client_(nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller"))
    , switch_caller_(nh)
  {
    if (!nh.hasParam("controllers_list"))
      ROS_ERROR("No controllers defined");
    ROS_INFO("Waiting for load_controller service...");
    load_client_.waitForExistence();
    ros::NodeHandle nh_list(nh, "controllers_list");
    XmlRpc::XmlRpcValue controllers;
    if (nh_list.getParam("state_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
      {
        state_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
    if (nh_list.getParam("main_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
      {
        main_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
    if (nh_list.getParam("calibration_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
      {
        calibration_controllers_.push_back(controllers[i]);
        loadController(controllers[i]);
      }
  }
  void update()
  {
    if (!switch_caller_.isCalling())
    {
      switch_caller_.startControllers(start_buffer_);
      switch_caller_.stopControllers(stop_buffer_);
      if (!start_buffer_.empty() || !stop_buffer_.empty())
      {
        switch_caller_.callService();
        start_buffer_.clear();
        stop_buffer_.clear();
      }
    }
  }
  void startController(const std::string& controller)
  {
    if (std::find(start_buffer_.begin(), start_buffer_.end(), controller) == start_buffer_.end())
      start_buffer_.push_back(controller);
    // AVoid setting controller to start and stop in the same time
    auto item = std::find(stop_buffer_.begin(), stop_buffer_.end(), controller);
    if (item != stop_buffer_.end())
      stop_buffer_.erase(item);
  }
  void stopController(const std::string& controller)
  {
    if (std::find(stop_buffer_.begin(), stop_buffer_.end(), controller) == stop_buffer_.end())
      stop_buffer_.push_back(controller);
    // AVoid setting controller to start and stop in the same time
    auto item = std::find(start_buffer_.begin(), start_buffer_.end(), controller);
    if (item != start_buffer_.end())
      start_buffer_.erase(item);
  }
  void startControllers(const std::vector<std::string>& controllers)
  {
    for (const auto& controller : controllers)
      startController(controller);
  }
  void stopControllers(const std::vector<std::string>& controllers)
  {
    for (const auto& controller : controllers)
      stopController(controller);
  }
  void startStateControllers()
  {
    startControllers(state_controllers_);
  }
  void startMainControllers()
  {
    startControllers(main_controllers_);
  }
  void stopMainControllers()
  {
    stopControllers(main_controllers_);
  }
  void startCalibrationControllers()
  {
    startControllers(calibration_controllers_);
  }
  void stopCalibrationControllers()
  {
    stopControllers(calibration_controllers_);
  }
  bool isCalling()
  {
    return switch_caller_.isCalling();
  }

private:
  void loadController(const std::string& controller)
  {
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

}  // namespace rm_common
