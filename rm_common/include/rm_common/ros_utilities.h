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
// Created by qiayuan on 7/7/20.
//

#pragma once

#include <ros/ros.h>
#include <XmlRpcException.h>

template <typename T>
T getParam(ros::NodeHandle& pnh, const std::string& param_name, const T& default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    const int tmp = value;
    return (double)tmp;
  }
  else
    return value;
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value, int field)
{
  ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
             (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
  XmlRpc::XmlRpcValue value_xml = value[field];
  return xmlRpcGetDouble(value[field]);
}

inline double xmlRpcGetDouble(XmlRpc::XmlRpcValue& value, const std::string& field, double default_value)
{
  if (value.hasMember(field))
  {
    ROS_ASSERT((value[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (value[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    return xmlRpcGetDouble(value[field]);
  }
  else
    return default_value;
}
