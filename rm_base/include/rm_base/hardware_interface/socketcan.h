//
// Created by qiayuan on 3/3/21.
//

#ifndef RM_BASE_HARDWARE_INTERFACE_SOCKETCAN_H
#define RM_BASE_HARDWARE_INTERFACE_SOCKETCAN_H

#include <linux/can.h>
#include <net/if.h>
// Multi-threading
#include <pthread.h>
#include <boost/function.hpp>

namespace can {

class SocketCAN {
 private:
  ifreq interface_request_{};
  sockaddr_can address_{};
  pthread_t receiver_thread_id_{};

 public:
  /**
   * CAN socket file descriptor
   */
  int sock_fd_ = -1;
  /**
   * Request for the child thread to terminate
   */
  bool terminate_receiver_thread_ = false;
  bool receiver_thread_running_ = false;

  SocketCAN() = default;
  ~SocketCAN();
  /**
   * Open and bind socket
   */
  bool open(const std::string &interface, boost::function<void(const can_frame &frame)> handler);
  /**
   * Close and unbind socket
   */
  void close();
  /**
   * Returns whether the socket is open or closed
   *
   * @retval true     Socket is open
   * @retval false    Socket is closed
   */
  bool is_open() const;
  /**
   * Sends the referenced frame to the bus
   */
  void write(can_frame *frame) const;
  /**
   * Starts a new thread, that will wait for socket events
   */
  bool start_receiver_thread();
  /**
   * Pointer to a function which shall be called
   * when frames are being received from the CAN bus
   */
  boost::function<void(const can_frame &frame)> reception_handler;
};

}

#endif //RM_BASE_HARDWARE_INTERFACE_SOCKETCAN_H
