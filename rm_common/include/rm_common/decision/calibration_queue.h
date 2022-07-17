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
// Created by qiayuan on 5/27/21.
//

#pragma once

#include "rm_common/decision/service_caller.h"
#include "rm_common/decision/controller_manager.h"

namespace rm_common
{
class CalibrationService
{
public:
  CalibrationService(XmlRpc::XmlRpcValue& rpc_value, ros::NodeHandle& nh)
  {
    ROS_ASSERT(rpc_value.hasMember("start_controllers"));
    ROS_ASSERT(rpc_value.hasMember("stop_controllers"));
    ROS_ASSERT(rpc_value.hasMember("services_name"));
    ROS_ASSERT(rpc_value["start_controllers"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(rpc_value["stop_controllers"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(rpc_value["services_name"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    start_controllers = getControllersName(rpc_value["start_controllers"]);
    stop_controllers = getControllersName(rpc_value["stop_controllers"]);
    for (int i = 0; i < rpc_value["services_name"].size(); ++i)
    {
      query_services.push_back(new QueryCalibrationServiceCaller(nh, rpc_value["services_name"][i]));
    }
  }
  void setCalibratedFalse()
  {
    for (auto& service : query_services)
      service->getService().response.is_calibrated = false;
  }
  bool isCalibrated()
  {
    bool is_calibrated = true;
    for (auto& service : query_services)
      is_calibrated &= service->isCalibrated();
    return is_calibrated;
  }
  void callService()
  {
    for (auto& service : query_services)
      service->callService();
  }
  std::vector<std::string> start_controllers, stop_controllers;
  std::vector<QueryCalibrationServiceCaller*> query_services;

private:
  static std::vector<std::string> getControllersName(XmlRpc::XmlRpcValue& rpc_value)
  {
    std::vector<std::string> controllers;
    for (int i = 0; i < rpc_value.size(); ++i)
    {
      controllers.push_back(rpc_value[i]);
    }
    return controllers;
  }
};

class CalibrationQueue
{
public:
  explicit CalibrationQueue(XmlRpc::XmlRpcValue& rpc_value, ros::NodeHandle& nh, ControllerManager& controller_manager)
    : controller_manager_(controller_manager), switched_(false)
  {
    // Don't calibration if using simulation
    ros::NodeHandle nh_global;
    bool use_sim_time;
    nh_global.param("use_sim_time", use_sim_time, false);
    if (use_sim_time || rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return;
    for (int i = 0; i < rpc_value.size(); ++i)
      calibration_services_.emplace_back(rpc_value[i], nh);
    last_query_ = ros::Time::now();
    calibration_itr_ = calibration_services_.end();
    // Start with calibrated, you should use reset() to start calibration.
  }
  void reset()
  {
    if (calibration_services_.empty())
      return;
    calibration_itr_ = calibration_services_.begin();
    switched_ = false;
    for (auto service : calibration_services_)
      service.setCalibratedFalse();
  }
  void update(const ros::Time& time, bool flip_controllers)
  {
    if (calibration_services_.empty())
      return;
    if (isCalibrated())
      return;
    if (switched_)
    {
      if (calibration_itr_->isCalibrated())
      {
        if (flip_controllers)
          controller_manager_.startControllers(calibration_itr_->stop_controllers);
        controller_manager_.stopControllers(calibration_itr_->start_controllers);
        calibration_itr_++;
        switched_ = false;
      }
      else if ((time - last_query_).toSec() > .2)
      {
        last_query_ = time;
        calibration_itr_->callService();
      }
    }
    else
    {
      // Switch controllers
      switched_ = true;
      if (calibration_itr_ != calibration_services_.end())
      {
        controller_manager_.startControllers(calibration_itr_->start_controllers);
        controller_manager_.stopControllers(calibration_itr_->stop_controllers);
      }
    }
  }
  void update(const ros::Time& time)
  {
    update(time, true);
  }
  bool isCalibrated()
  {
    return calibration_itr_ == calibration_services_.end();
  }
  void stopController()
  {
    if (calibration_services_.empty())
      return;
    if (calibration_itr_ != calibration_services_.end() && switched_)
      controller_manager_.stopControllers(calibration_itr_->stop_controllers);
  }
  void stop()
  {
    if (switched_)
    {
      calibration_itr_ = calibration_services_.end();
      switched_ = false;
    }
  }

private:
  ros::Time last_query_;
  std::vector<CalibrationService> calibration_services_;
  std::vector<CalibrationService>::iterator calibration_itr_;
  ControllerManager& controller_manager_;
  bool switched_;
};
}  // namespace rm_common
