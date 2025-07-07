//
// CAN Bus implementation for ROS2
//

#include "actuator_interface/can_manager/can_interface/can_bus.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

namespace can_interface {

CanBus::CanBus(const std::string& device_name, int thread_priority)
    : device_name_(device_name)
    , thread_priority_(thread_priority)
    , socket_fd_(-1)
    , is_open_(false)
    , logger_(rclcpp::get_logger("can_bus_" + device_name))
{
}

CanBus::~CanBus() {
    close();
}

bool CanBus::open() {
    if (is_open_) {
        RCLCPP_WARN(logger_, "CAN bus %s is already open", device_name_.c_str());
        return true;
    }

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to create CAN socket for %s", device_name_.c_str());
        return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, device_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "Failed to get interface index for %s", device_name_.c_str());
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "Failed to bind CAN socket for %s", device_name_.c_str());
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    is_open_ = true;
    RCLCPP_INFO(logger_, "CAN bus %s opened successfully", device_name_.c_str());
    return true;
}

void CanBus::close() {
    if (is_open_ && socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        is_open_ = false;
        RCLCPP_INFO(logger_, "CAN bus %s closed", device_name_.c_str());
    }
}

bool CanBus::isOpen() const {
    return is_open_;
}

bool CanBus::write(const can_frame& frame) {
    if (!is_open_) {
        RCLCPP_ERROR(logger_, "CAN bus %s is not open", device_name_.c_str());
        return false;
    }

    ssize_t bytes_sent = ::write(socket_fd_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        RCLCPP_ERROR(logger_, "Failed to write CAN frame to %s", device_name_.c_str());
        return false;
    }

    return true;
}

bool CanBus::read(can_frame& frame) {
    if (!is_open_) {
        RCLCPP_ERROR(logger_, "CAN bus %s is not open", device_name_.c_str());
        return false;
    }

    ssize_t bytes_read = ::read(socket_fd_, &frame, sizeof(frame));
    if (bytes_read != sizeof(frame)) {
        return false;  // No data available or error
    }

    return true;
}

bool CanBus::readBuffer(std::vector<CanFrameStamp>& buffer, int max_frames) {
    if (!is_open_) {
        return false;
    }

    buffer.clear();
    buffer.reserve(max_frames);

    for (int i = 0; i < max_frames; ++i) {
        can_frame frame;
        if (read(frame)) {
            buffer.emplace_back(frame, rclcpp::Clock().now());
        } else {
            break;  // No more frames available
        }
    }

    return !buffer.empty();
}

} // namespace can_interface 