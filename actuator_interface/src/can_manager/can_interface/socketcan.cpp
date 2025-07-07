//
// SocketCAN implementation for ROS2
//

#include "actuator_interface/can_manager/can_interface/socketcan.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>

namespace can_interface {

SocketCAN::SocketCAN(const std::string& device_name, int thread_priority)
    : CanBus(device_name, thread_priority)
{
}

bool SocketCAN::open() {
    return CanBus::open();
}

void SocketCAN::close() {
    CanBus::close();
}

bool SocketCAN::setupSocket() {
    // This is handled in the base class CanBus::open()
    return true;
}

bool SocketCAN::bindSocket() {
    // This is handled in the base class CanBus::open()
    return true;
}

bool SocketCAN::setSocketOptions() {
    // Additional socket options can be set here if needed
    // For now, use default options from base class
    return true;
}

} // namespace can_interface 