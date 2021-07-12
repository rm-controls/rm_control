//
// Created by qiayuan on 5/27/21.
//

#ifndef RM_COMMON_CALIBRATION_MANAGER_H_
#define RM_COMMON_CALIBRATION_MANAGER_H_
#include "rm_common/decision/service_caller.h"

namespace rm_common {
struct CalibrationService {
  SwitchControllersService *switch_services_;
  QueryCalibrationService *query_services_;
};

class CalibrationManager {
 public:
  explicit CalibrationManager(ros::NodeHandle &nh) {
    // Don't calibration if using simulation
    ros::NodeHandle nh_global;
    bool use_sim_time;
    nh_global.param("use_sim_time", use_sim_time, false);
    if (use_sim_time)
      return;
    XmlRpc::XmlRpcValue rpc_value;
    if (!nh.getParam("calibration_manager", rpc_value) || rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_INFO("No calibration controllers defined");
      return;
    }
    for (int i = 0; i < rpc_value.size(); ++i) {
      ROS_ASSERT(rpc_value[i].hasMember("switch"));
      ROS_ASSERT(rpc_value[i].hasMember("query"));
      calibration_services_.push_back(CalibrationService{
          .switch_services_ = new SwitchControllersService(rpc_value[i]["switch"]),
          .query_services_ = new QueryCalibrationService(rpc_value[i]["query"])});
    }
    last_query_ = ros::Time::now();
    // Start with calibrated, you should use reset() to start calibration.
    calibration_itr_ = calibration_services_.end();
  }
  void reset() {
    if (calibration_services_.empty())
      return;
    calibration_itr_ = calibration_services_.begin();
    for (auto service:calibration_services_) {
      service.switch_services_->getService().response.ok = false;
      service.query_services_->getService().response.is_calibrated = false;
    }
  }
  void checkCalibrate(const ros::Time &time) {
    if (calibration_services_.empty())
      return;
    if (isCalibrated())
      return;
    if (calibration_itr_->switch_services_->getOk()) {
      if (calibration_itr_->query_services_->getIsCalibrated()) {
        calibration_itr_->switch_services_->flipControllers();
        calibration_itr_->switch_services_->callService();
        calibration_itr_++;
      } else if ((time - last_query_).toSec() > .2) {
        last_query_ = time;
        calibration_itr_->query_services_->callService();
      }
    } else {
      calibration_itr_->switch_services_->switchControllers();
      calibration_itr_->switch_services_->callService();
    }
  }
 private:
  bool isCalibrated() { return calibration_itr_ == calibration_services_.end(); }
  ros::Time last_query_;
  std::vector<CalibrationService> calibration_services_;
  std::vector<CalibrationService>::iterator calibration_itr_;
};
}

#endif //RM_COMMON_CALIBRATION_MANAGER_H_
