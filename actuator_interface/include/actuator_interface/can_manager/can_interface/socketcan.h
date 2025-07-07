//
// SocketCAN interface for ROS2
//

#pragma once

#include "can_bus.h"
#include <rclcpp/rclcpp.hpp>

namespace can_interface {

class SocketCAN : public CanBus {
public:
  explicit SocketCAN(const std::string& device_name, int thread_priority = 0);
  ~SocketCAN() override = default;

  bool open() override;
  void close() override;

private:
  bool setupSocket();
  bool bindSocket();
  bool setSocketOptions();
};

} // namespace can_interface 