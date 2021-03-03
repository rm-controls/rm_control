//
// Created by qiayuan on 3/3/21.
//
#include "rm_base/hardware_interface/socketcan.h"
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <utility>

namespace can {

/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketCAN::~SocketCAN() {
  printf("Destroying SocketCAN adapter...\n");
  if (this->is_open())
    this->close();
}

bool SocketCAN::open(char *interface, boost::function<void(const can_frame &frame)> handler) {
  reception_handler = std::move(handler);
  // Request a socket
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ == -1) {
    printf("Error: Unable to create a CAN socket\n");
    return false;
  }
  printf("Created CAN socket with descriptor %d.\n", sock_fd_);

  // Get the index of the network interface
  strncpy(interface_request_.ifr_name, interface, IFNAMSIZ);
  if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1) {
    printf("Unable to select CAN interface %s: I/O control error\n", interface);
    // Invalidate unusable socket
    close();
    return false;
  }
  printf("Found: %s has interface index %d.\n", interface, interface_request_.ifr_ifindex);

  // Bind the socket to the network interface
  address_.can_family = AF_CAN;
  address_.can_ifindex = interface_request_.ifr_ifindex;
  int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&address_), sizeof(address_));
  if (rc == -1) {
    printf("Failed to bind socket to network interface\n");
    close();
    return false;
  }
  printf("Successfully bound socket to interface %d.\n", interface_request_.ifr_ifindex);
  // Start a separate, event-driven thread for frame reception
  return start_receiver_thread();
}

void SocketCAN::close() {
  terminate_receiver_thread_ = true;
  while (receiver_thread_running_);

  if (!is_open())
    return;

  // Close the file descriptor for our socket
  ::close(sock_fd_);
  sock_fd_ = -1;

  printf("CAN socket destroyed.\n");
}

bool SocketCAN::is_open() const {
  return (sock_fd_ != -1);
}

void SocketCAN::wirte(can_frame *frame) const {
  if (!is_open()) {
    printf("Unable to wirte: Socket not open\n");
    return;
  }
  if (write(sock_fd_, frame, sizeof(can_frame)) == -1) {
    printf("Unable to wirte\n");
  }
}

static void *socketcan_receiver_thread(void *argv) {
  /*
   * The first and only argument to this function
   * is the pointer to the object, which started the thread.
   */
  auto *sock = (SocketCAN *) argv;
  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;
  // Highest file descriptor in set
  int maxfd = sock->sock_fd_;
  // How long 'select' shall wait before returning with timeout
  struct timeval timeout{};
  // Buffer to store incoming frame
  can_frame rx_frame{};
  // Run until termination signal received
  sock->receiver_thread_running_ = true;
  while (!sock->terminate_receiver_thread_) {
    timeout.tv_sec = 1.; // Should be set each loop
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(sock->sock_fd_, &descriptors);
    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout)) {
      int len = read(sock->sock_fd_, &rx_frame, CAN_MTU);
      if (len < 0)
        continue;
      if (sock->reception_handler != nullptr)
        sock->reception_handler(rx_frame);
    }
  }
  sock->receiver_thread_running_ = false;
  return nullptr;
}

bool SocketCAN::start_receiver_thread() {
  // Frame reception is accomplished in a separate, event-driven thread.
  // See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
  terminate_receiver_thread_ = false;
  int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
  if (rc != 0) {
    printf("Unable to start receiver thread.\n");
    return false;
  }
  printf("Successfully started receiver thread with ID %d.\n", (int) receiver_thread_id_);
  return true;
}

}