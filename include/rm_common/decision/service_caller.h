//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_COMMON_SERVICE_CALLER_H_
#define RM_COMMON_SERVICE_CALLER_H_

#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
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
  explicit ServiceCallerBase(ros::NodeHandle &nh, const std::string &service_name = "")
      : fail_count_(0), fail_limit_(0) {
    nh.param("fail_limit", fail_limit_, 0);
    if (!nh.param("service_name", service_name_, service_name) && service_name.empty()) {
      ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
      return;
    }
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ServiceCallerBase(XmlRpc::XmlRpcValue &controllers, ros::NodeHandle &nh,
                    const std::string &service_name = "") : fail_count_(0), fail_limit_(0) {
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
    while (!client_.call(service_)) {
      ROS_INFO_ONCE("Failed to call service %s on %s. Retrying now ...",
                    typeid(ServiceType).name(), service_name_.c_str());
      if (fail_limit_ != 0) {
        fail_count_++;
        if (fail_count_ >= fail_limit_) {
          ROS_ERROR("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_.c_str());
          fail_count_ = 0;
        }
      }
    }
  }
  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_;
  std::thread *thread_{};
  std::mutex mutex_;
  int fail_count_, fail_limit_;
};

class SwitchControllersServiceCaller : public ServiceCallerBase<controller_manager_msgs::SwitchController> {
 public:
  explicit SwitchControllersServiceCaller(ros::NodeHandle &nh) :
      ServiceCallerBase<controller_manager_msgs::SwitchController>(nh, "/controller_manager/switch_controller") {
    service_.request.strictness = service_.request.BEST_EFFORT;
    service_.request.start_asap = true;
  }
  void startControllers(const std::vector<std::string> &controllers) {
    service_.request.start_controllers = controllers;
  }
  void stopControllers(const std::vector<std::string> &controllers) {
    service_.request.stop_controllers = controllers;
  }
  bool getOk() {
    if (isCalling()) return false;
    return service_.response.ok;
  }
};

class QueryCalibrationService : public ServiceCallerBase<control_msgs::QueryCalibrationState> {
 public:
  explicit QueryCalibrationService(ros::NodeHandle &nh) : ServiceCallerBase<control_msgs::QueryCalibrationState>(nh) {}
  QueryCalibrationService(XmlRpc::XmlRpcValue &controllers, ros::NodeHandle &nh)
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
  void setEnemyColor(const RefereeData &referee_data) {
    if (referee_data.robot_id_ != 0 && !is_set_) {
      //RED:1~9  BLUE:101~109
      service_.request.color = referee_data.robot_color_ == "blue" ? "red" : "blue";
      callService();
      if (getIsSwitch())
        is_set_ = true;
    }
  }
  void switchEnemyColor() {
    if (is_set_)
      service_.request.color = service_.request.color == "blue" ? "red" : "blue";
  }
  std::string getColor() {
    return service_.request.color;
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
