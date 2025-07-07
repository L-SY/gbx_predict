//
// RM Actuator for ROS2 - Simplified effort-only interface
//

#pragma once

#include "can_actuator.h"
#include <stdexcept>

namespace device {

/**
 * @brief RM series actuator implementation (RM2006, RM3508, etc.)
 * 
 * This class provides a simplified interface focused on effort control.
 * Higher-level control (position/velocity) should be implemented in ros2_control controllers.
 * 
 * Features:
 * - Raw CAN communication with RM actuators
 * - Multi-turn position tracking
 * - Effort (torque current) commands
 * - Position, velocity, effort state feedback
 */
class RmActuator : public CanActuator {
public:
  RmActuator(const std::string& name,
             const std::string& bus,
             int id,
             const std::string& motor_type,
             const YAML::Node& config);

  ~RmActuator() override = default;

  // Device lifecycle
  can_frame start() override;
  can_frame close() override;
  
  // Communication
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override;
  can_frame write() override;

  // Configuration
  ActuatorCoefficients getCoefficientsFor() const override;

private:
  // Configuration parameters
  int16_t max_effort_{10000};  // Maximum effort output
  uint32_t rx_id_{0x201};      // CAN receive ID
  uint32_t tx_id_{0x200};      // CAN transmit ID
  uint8_t motor_index_{0};     // Motor index for multi-motor frames
  
  // Raw sensor values
  uint16_t encoder_raw_{0};
  uint16_t encoder_last_{0};
  int16_t velocity_raw_{0};
  int16_t current_raw_{0};
  uint8_t temperature_{0};
  
  // Multi-turn position tracking
  int32_t turn_count_{0};
  uint32_t seq_count_{0};
  
  // Helper methods
  void parseCanFrame(const can_frame& frame);
  void updateMultiTurnPosition();
  can_frame generateCommandFrame();
  void validateConfiguration(const YAML::Node& config);
};

} // namespace device 