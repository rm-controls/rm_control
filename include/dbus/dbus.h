//
// Created by qiayuan on 2019/10/30.
//

#ifndef SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_
#define SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_

#include <cstdint>
#include <rm_msgs/DbusData.h>

typedef struct {
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  uint8_t s0;
  uint8_t s1;
  int16_t wheel;

  int16_t x;
  int16_t y;
  int16_t z;

  uint8_t l;
  uint8_t r;
  uint16_t key;

} DBusData_t;

class DBus {

 public:
  DBus() = default;
  ~DBus() = default;
  void init(const char *serial);
  void getData(rm_msgs::DbusData *d_bus_data);
  void read();

 private:
  DBusData_t d_bus_data_{};
  int port_{};
  int16_t buff_[18]{};
  bool is_success{};
  bool is_updata_ = false;
  void unpack();
};

#endif //SRC_RM_BRIDGE_INCLUDE_RT_RT_DBUS_H_

