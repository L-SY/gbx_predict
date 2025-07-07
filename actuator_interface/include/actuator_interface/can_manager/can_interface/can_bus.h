//
// CAN Bus interface for ROS2
//

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <memory>

namespace can_interface {

struct CanFrameStamp {
  can_frame frame;
  rclcpp::Time stamp;
  
  CanFrameStamp() : stamp(rclcpp::Clock().now()) {}
  CanFrameStamp(const can_frame& f, const rclcpp::Time& t) : frame(f), stamp(t) {}
};

class CanBus {
public:
  explicit CanBus(const std::string& device_name, int thread_priority = 0);
  virtual ~CanBus();

  virtual bool open();
  virtual void close();
  bool isOpen() const;

  bool write(const can_frame& frame);
  bool read(can_frame& frame);
  bool readBuffer(std::vector<CanFrameStamp>& buffer, int max_frames = 100);

  std::string getDeviceName() const { return device_name_; }
  int getThreadPriority() const { return thread_priority_; }

private:
  std::string device_name_;
  int thread_priority_;
  int socket_fd_;
  bool is_open_;
  
  rclcpp::Logger logger_;
};

} // namespace can_interface 