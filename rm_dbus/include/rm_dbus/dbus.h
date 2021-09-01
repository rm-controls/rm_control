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
// Created by qiayuan on 2019/10/30.
//

#ifndef SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_
#define SRC_RM_BASE_INCLUDE_RT_RT_DBUS_H_

#include <cstdint>
#include <rm_msgs/DbusData.h>

typedef struct
{
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

class DBus
{
public:
  DBus() = default;
  ~DBus() = default;
  void init(const char* serial);
  void getData(rm_msgs::DbusData* d_bus_data) const;
  void read();

private:
  DBusData_t d_bus_data_{};
  int port_{};
  int16_t buff_[18]{};
  bool is_success{};
  bool is_update_ = false;
  void unpack();
};

#endif  // SRC_RM_BRIDGE_INCLUDE_RT_RT_DBUS_H_
