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
// Created by qiayuan on 3/3/21.
//
#include "rm_hw/hardware_interface/socketcan.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <utility>
#include <ros/ros.h>

namespace can
{
/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketCAN::~SocketCAN()
{
  if (this->is_open())
    this->close();
}

bool SocketCAN::open(const std::string& interface, boost::function<void(const can_frame& frame)> handler)
{
  reception_handler = std::move(handler);
  // Request a socket
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ == -1)
  {
    ROS_ERROR("Error: Unable to create a CAN socket");
    return false;
  }
  char name[16] = {};  // avoid stringop-truncation
  strncpy(name, interface.c_str(), interface.size());
  strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
  // Get the index of the network interface
  if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1)
  {
    ROS_ERROR("Unable to select CAN interface %s: I/O control error", name);
    // Invalidate unusable socket
    close();
    return false;
  }
  // Bind the socket to the network interface
  address_.can_family = AF_CAN;
  address_.can_ifindex = interface_request_.ifr_ifindex;
  int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
  if (rc == -1)
  {
    ROS_ERROR("Failed to bind socket to %s network interface", name);
    close();
    return false;
  }
  // Start a separate, event-driven thread for frame reception
  return start_receiver_thread();
}

void SocketCAN::close()
{
  terminate_receiver_thread_ = true;
  while (receiver_thread_running_)
    ;

  if (!is_open())
    return;
  // Close the file descriptor for our socket
  ::close(sock_fd_);
  sock_fd_ = -1;
}

bool SocketCAN::is_open() const
{
  return (sock_fd_ != -1);
}

void SocketCAN::write(can_frame* frame) const
{
  if (!is_open())
  {
    ROS_ERROR_THROTTLE(5., "Unable to write: Socket %s not open", interface_request_.ifr_name);
    return;
  }
  if (::write(sock_fd_, frame, sizeof(can_frame)) == -1)
    ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);
}

static void* socketcan_receiver_thread(void* argv)
{
  /*
   * The first and only argument to this function
   * is the pointer to the object, which started the thread.
   */
  auto* sock = (SocketCAN*)argv;
  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;
  // Highest file descriptor in set
  int maxfd = sock->sock_fd_;
  // How long 'select' shall wait before returning with timeout
  struct timeval timeout
  {
  };
  // Buffer to store incoming frame
  can_frame rx_frame{};
  // Run until termination signal received
  sock->receiver_thread_running_ = true;
  while (!sock->terminate_receiver_thread_)
  {
    timeout.tv_sec = 1.;  // Should be set each loop
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(sock->sock_fd_, &descriptors);
    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      size_t len = read(sock->sock_fd_, &rx_frame, CAN_MTU);
      if (len < 0)
        continue;
      if (sock->reception_handler != nullptr)
        sock->reception_handler(rx_frame);
    }
  }
  sock->receiver_thread_running_ = false;
  return nullptr;
}

bool SocketCAN::start_receiver_thread()
{
  // Frame reception is accomplished in a separate, event-driven thread.
  // See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
  terminate_receiver_thread_ = false;
  int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
  if (rc != 0)
  {
    ROS_ERROR("Unable to start receiver thread");
    return false;
  }
  ROS_INFO("Successfully started receiver thread with ID %lu", receiver_thread_id_);
  return true;
}

}  // namespace can
