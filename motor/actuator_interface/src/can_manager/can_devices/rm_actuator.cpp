//
// RM Actuator implementation for ROS2 - Simplified effort interface
//

#include "actuator_interface/can_manager/can_devices/rm_actuator.h"
#include "actuator_interface/can_manager/can_interface/can_bus.h"
#include "actuator_interface/common/lp_filter.h"
#include <cstring>
#include <algorithm>
#include <cmath>

namespace device {

RmActuator::RmActuator(const std::string& name,
                       const std::string& bus,
                       int id,
                       const std::string& motor_type,
                       const YAML::Node& config)
    : CanActuator(name, bus, id, motor_type, DeviceType::read_write, config) {
  coeff_ = getCoefficientsFor();
  if (config["max_out"]) {
    int tmp = static_cast<int>(config["max_out"].as<double>());
    max_out_ = static_cast<double>(tmp);
  } else {
    throw std::invalid_argument("Missing max_out in config");
  }
}

can_frame RmActuator::start() { 
  is_halted_ = false; 
  return can_frame{}; 
}

can_frame RmActuator::close() { 
  return can_frame{}; 
}

void RmActuator::read(const can_interface::CanFrameStamp &frameStamp) {
  const can_frame &frame = frameStamp.frame;
  if ((frameStamp.stamp - last_timestamp_).seconds() < 0.0005)
  {
    return;
  }

  q_raw_ = (frame.data[0] << 8u) | frame.data[1];
  qd_raw_ = (frame.data[2] << 8u) | frame.data[3];
  int16_t cur = (frame.data[4] << 8u) | frame.data[5];
  temp_ = frame.data[6];

  // Multiple circle
  if (seq_ != 0)  // not the first receive
  {
    if (q_raw_ - q_raw_last_ > 4096)
      q_circle_--;
    else if (q_raw_ - q_raw_last_ < -4096)
      q_circle_++;
  }

  position_ = coeff_.act2pos * static_cast<double>(q_raw_ + 8191 * q_circle_) + coeff_.pos_offset;
  velocity_ = coeff_.act2vel * static_cast<double>(qd_raw_) ;
  effort_   =  coeff_.act2effort * static_cast<double>(cur);
  // Low pass filter
  lp_filter_.input(velocity_, frameStamp.stamp);
  velocity_ = lp_filter_.output();

  rclcpp::Time current_time = frameStamp.stamp;
  updateFrequency(current_time);
  q_raw_last_ = q_raw_;
  seq_++;
}

void RmActuator::readBuffer(
    const std::vector<can_interface::CanFrameStamp> &frameStamps) {
  for (const auto &frameStamp : frameStamps) {
    if (frameStamp.frame.can_id == master_id_) {
        read(frameStamp);
        break;
    }
  }
}

can_frame RmActuator::write() {
  can_frame frame{};
  if (is_halted_)
    return frame;

  frame.can_dlc = 8;
  double cmd = minAbs(coeff_.effort2act * cmd_effort_, max_out_);

  int index = id_ -1 ;
  if (-1 < index && index < 4)
  {
    frame.can_id = 0x200;
    frame.data[2 * index] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
    frame.data[2 * index + 1] = static_cast<uint8_t>(cmd);
  }
  else if (3 < index && index < 8)
  {
    frame.can_id = 0x1FF;
    frame.data[2 * (index - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
    frame.data[2 * (index - 4) + 1] = static_cast<uint8_t>(cmd);
  }

  return frame;
}

ActuatorCoefficients RmActuator::getCoefficientsFor() const {
    if (model_ == "RM3508") {
        return ActuatorCoefficients(0.0007669903, 0.1047197551, 1.90702994e-5, 0.0,
                                    0.0, 0.0, 0.0, 52437.561519, 0.0, 0.0, 0.0);
    } else if (model_ == "RM6020") {
        return ActuatorCoefficients(0.0007670840, 0.1047197551, 5.880969e-5, 0.0,
                                    0.0, 0.0, 0.0, 25000.0, 0.0, 0.0, 0.0);
    } else if (model_ == "RM2006") {
        return ActuatorCoefficients(2.13078897e-5, 0.0029088820, 0.00018, 0.0, 0.0,
                                    0.0, 0.0, 5555.5555555, 0.0, 0.0, 0.0);
    } else {
        throw std::invalid_argument("RmActuator: unsupported motor_type → " + model_);
    }
}



} // namespace device 