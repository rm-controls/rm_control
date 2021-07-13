//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_COMMON_SERVICE_CALLER_H_
#define RM_COMMON_SERVICE_CALLER_H_

#include <chrono>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/service.h>
#include <controller_manager_msgs/SwitchController.h>
#include <control_msgs/QueryCalibrationState.h>
#include <rm_msgs/ColorSwitch.h>
#include <rm_msgs/TargetSwitch.h>

namespace rm_common {
template<class ServiceType>
class ServiceCallerBase {
 public:
  ServiceCallerBase() = default;
  explicit ServiceCallerBase(ros::NodeHandle &nh, const std::string &service_name = "") {
    if (!nh.param("service_name", service_name_, service_name))
      if (service_name.empty()) {
        ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
        return;
      }
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ServiceCallerBase(const XmlRpc::XmlRpcValue &controllers, ros::NodeHandle &nh,
                    const std::string &service_name = "") {
    if (controllers.hasMember("service_name"))
      service_name_ = static_cast<std::string>(controllers["service_name"]);
    else {
      service_name_ = service_name;
      if (service_name.empty()) {
        ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
        return;
      }
    }
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ~ServiceCallerBase() { delete thread_; }
  void callService() {
    if (isCalling())
      return;
    thread_ = new std::thread(&ServiceCallerBase::callingThread, this);
    thread_->detach();
  }
  ServiceType &getService() { return service_; }
  bool isCalling() {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }
 protected:
  void callingThread() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!client_.call(service_))
      ROS_ERROR("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_.c_str());
  }

  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_;
  std::thread *thread_{};
  std::mutex mutex_;
};

class SwitchControllersService : public ServiceCallerBase<controller_manager_msgs::SwitchController> {
 public:
  explicit SwitchControllersService(ros::NodeHandle &nh) : ServiceCallerBase<controller_manager_msgs::SwitchController>(
      nh, "/controller_manager/switch_controller") {
    XmlRpc::XmlRpcValue controllers;
    if (nh.getParam("start_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        start_controllers_.push_back(controllers[i]);
    if (nh.getParam("stop_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        stop_controllers_.push_back(controllers[i]);
    if (start_controllers_.empty() && stop_controllers_.empty())
      ROS_ERROR("No start/stop controllers specified (namespace: %s)", nh.getNamespace().c_str());
    service_.request.strictness = service_.request.BEST_EFFORT;
    service_.request.start_asap = true;
  }
  SwitchControllersService(const XmlRpc::XmlRpcValue &controllers, ros::NodeHandle &nh)
      : ServiceCallerBase<controller_manager_msgs::SwitchController>(
      controllers, nh, "/controller_manager/switch_controller") {
    if (controllers.hasMember("start_controllers"))
      for (int i = 0; i < controllers.size(); ++i)
        start_controllers_.push_back(controllers["start_controllers"][i]);
    if (controllers.hasMember("stop_controllers"))
      for (int i = 0; i < controllers.size(); ++i)
        stop_controllers_.push_back(controllers["stop_controllers"][i]);
    if (start_controllers_.empty() && stop_controllers_.empty())
      ROS_ERROR("No start/stop controllers specified (namespace: /rm_manual/calibration_manager)");
    service_.request.strictness = service_.request.BEST_EFFORT;
    service_.request.start_asap = true;
  }
  void startControllersOnly() {
    service_.request.start_controllers = start_controllers_;
    service_.request.stop_controllers.clear();
  }
  void stopControllersOnly() {
    service_.request.stop_controllers = stop_controllers_;
    service_.request.start_controllers.clear();
  }
  void switchControllers() {
    service_.request.start_controllers = start_controllers_;
    service_.request.stop_controllers = stop_controllers_;
  }
  void flipControllers() {
    service_.request.start_controllers = stop_controllers_;
    service_.request.stop_controllers = start_controllers_;
  }
  bool getOk() {
    if (isCalling()) return false;
    return service_.response.ok;
  }
 private:
  std::vector<std::string> start_controllers_, stop_controllers_;
};

class QueryCalibrationService : public ServiceCallerBase<control_msgs::QueryCalibrationState> {
 public:
  explicit QueryCalibrationService(ros::NodeHandle &nh) : ServiceCallerBase<control_msgs::QueryCalibrationState>(nh) {}
  QueryCalibrationService(const XmlRpc::XmlRpcValue &controllers, ros::NodeHandle &nh)
      : ServiceCallerBase<control_msgs::QueryCalibrationState>(controllers, nh) {}
  bool getIsCalibrated() {
    if (isCalling()) return false;
    return service_.response.is_calibrated;
  }
};

class SwitchEnemyColorService : public ServiceCallerBase<rm_msgs::ColorSwitch> {
 public:
  explicit SwitchEnemyColorService(ros::NodeHandle &nh) : ServiceCallerBase<rm_msgs::ColorSwitch>(
      nh, "/detection/enemy_color_change") {}
  void setEnemyColor(const Referee &referee) {
    if (referee.robot_id_ != 0 && !is_set_) {
      //RED:1~9  BLUE:101~109
      service_.request.color = referee.robot_id_ < 50 ? "blue" : "red";
      callService();
      if (getIsSwitch())
        is_set_ = true;
    }
  }
  void SwitchEnemyColor() {
    if (is_set_)
      service_.request.color = service_.request.color == "blue" ? "red" : "blue";
  }
  bool getIsSwitch() {
    if (isCalling()) return false;
    return service_.response.is_success;
  }

 private:
  bool is_set_{};
};

class SwitchTargetTypeService : public ServiceCallerBase<rm_msgs::TargetSwitch> {
 public:
  explicit SwitchTargetTypeService(ros::NodeHandle &nh) : ServiceCallerBase<rm_msgs::TargetSwitch>(
      nh, "/detection/target_change") {
    service_.request.target = "armor";
  }
  void switchTargetType() {
    service_.request.target = service_.request.target == "armor" ? "buff" : "armor";
  }
  std::string getTarget() {
    return service_.request.target;
  }
  bool getIsSwitch() {
    if (isCalling()) return false;
    return service_.response.target_switch_is_success;
  }
};

}

#endif //RM_COMMON_SERVICE_CALLER_H_
