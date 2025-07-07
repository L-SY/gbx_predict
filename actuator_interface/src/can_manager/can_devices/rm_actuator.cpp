//
// RM Actuator implementation for ROS2 - Simplified effort interface
//

#include "actuator_interface/can_manager/can_devices/rm_actuator.h"
#include "actuator_interface/can_manager/can_interface/can_bus.h"
#include <cstring>
#include <algorithm>
#include <cmath>

namespace device {

RmActuator::RmActuator(const std::string& name,
                       const std::string& bus,
                       int id,
                       const std::string& motor_type,
                       const YAML::Node& config)
    : CanActuator(name, bus, id, motor_type, DeviceType::read_write, config)
{
    // Validate and load configuration
    validateConfiguration(config);
    
    // Set motor-specific parameters
    if (config["max_effort"]) {
        max_effort_ = config["max_effort"].as<int16_t>();
    }
    
    if (config["rx_id"]) {
        rx_id_ = config["rx_id"].as<uint32_t>();
    }
    
    if (config["tx_id"]) {
        tx_id_ = config["tx_id"].as<uint32_t>();
    }
    
    if (config["motor_index"]) {
        motor_index_ = config["motor_index"].as<uint8_t>();
    }
    
    // Calculate motor index from rx_id if not specified
    if (!config["motor_index"]) {
        if (rx_id_ >= 0x201 && rx_id_ <= 0x204) {
            motor_index_ = rx_id_ - 0x201;
            tx_id_ = 0x200;
        } else if (rx_id_ >= 0x205 && rx_id_ <= 0x208) {
            motor_index_ = rx_id_ - 0x205;
            tx_id_ = 0x1FF;
        }
    }
    
    RCLCPP_INFO(logger_, "RM Actuator %s initialized: rx_id=0x%X, tx_id=0x%X, motor_index=%d, max_effort=%d", 
                name.c_str(), rx_id_, tx_id_, motor_index_, max_effort_);
}

can_frame RmActuator::start() {
    is_halted_ = false;
    RCLCPP_INFO(logger_, "Starting RM actuator: %s", name_.c_str());
    return can_frame{};  // RM actuators don't need specific start frames
}

can_frame RmActuator::close() {
    is_halted_ = true;
    RCLCPP_INFO(logger_, "Closing RM actuator: %s", name_.c_str());
    
    // Send zero effort command to stop motor safely
    cmd_effort_ = 0.0;
    return generateCommandFrame();
}

void RmActuator::read(const can_interface::CanFrameStamp& frameStamp) {
    const can_frame& frame = frameStamp.frame;
    
    // Check if this frame is for our motor
    if (frame.can_id != rx_id_) {
        return;
    }
    
    // Rate limiting to prevent excessive processing
    if ((frameStamp.stamp - last_timestamp_).seconds() < 0.0005) {
        return;  // Skip updates faster than 2kHz
    }
    
    // Parse the CAN frame
    parseCanFrame(frame);
    
    // Update multi-turn position
    updateMultiTurnPosition();
    
    // Convert raw values to physical units
    ActuatorCoefficients coeff = getCoefficientsFor();
    
    double total_encoder = static_cast<double>(encoder_raw_ + 8192 * turn_count_);
    position_ = coeff.act2pos * total_encoder + coeff.pos_offset;
    velocity_ = coeff.act2vel * static_cast<double>(velocity_raw_);
    effort_ = coeff.act2effort * static_cast<double>(current_raw_);
    
    // Apply low-pass filter to velocity
    lp_filter_.input(velocity_, frameStamp.stamp);
    velocity_ = lp_filter_.output();
    
    // Update timing and sequence
    updateFrequency(frameStamp.stamp);
    seq_count_++;
    
    // Debug logging (throttled)
    if (seq_count_ % 1000 == 0) {
        RCLCPP_DEBUG(logger_, "RM %s: pos=%.3f, vel=%.3f, eff=%.3f, temp=%d°C", 
                    name_.c_str(), position_, velocity_, effort_, temperature_);
    }
}

void RmActuator::readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) {
    for (const auto& frameStamp : buffer) {
        if (frameStamp.frame.can_id == rx_id_) {
            read(frameStamp);
        }
    }
}

can_frame RmActuator::write() {
    if (is_halted_) {
        // Send zero effort when halted
        cmd_effort_ = 0.0;
    }
    
    return generateCommandFrame();
}

ActuatorCoefficients RmActuator::getCoefficientsFor() const {
    ActuatorCoefficients coeff;
    
    // RM series motor specifications
    // Position: 8192 counts per revolution for 14-bit encoders
    coeff.act2pos = (2.0 * M_PI) / 8192.0;  // rad per count
    coeff.pos2act = 8192.0 / (2.0 * M_PI);  // count per rad
    
    // Velocity: directly in RPM from motor
    coeff.act2vel = (2.0 * M_PI) / 60.0;    // rad/s per RPM
    coeff.vel2act = 60.0 / (2.0 * M_PI);    // RPM per rad/s
    
    // Effort: depends on motor model
    // RM2006: ~1.2 N⋅m rated torque, ~10A rated current
    // Using current as effort proxy (torque = Kt * current)
    if (model_.find("RM2006") != std::string::npos) {
        coeff.act2effort = 1.2 / 10000.0;       // N⋅m per current unit
        coeff.effort2act = 10000.0 / 1.2;       // current unit per N⋅m
    } else if (model_.find("RM3508") != std::string::npos) {
        coeff.act2effort = 3.0 / 16384.0;       // Higher torque motor
        coeff.effort2act = 16384.0 / 3.0;
    } else {
        // Default values
        coeff.act2effort = 1.0 / 10000.0;
        coeff.effort2act = 10000.0;
    }
    
    // No offsets for RM motors
    coeff.pos_offset = 0.0;
    coeff.vel_offset = 0.0;
    coeff.effort_offset = 0.0;
    
    return coeff;
}

void RmActuator::parseCanFrame(const can_frame& frame) {
    // RM motor feedback format:
    // Byte 0-1: Encoder angle (0-8191)
    // Byte 2-3: Velocity in RPM
    // Byte 4-5: Torque current
    // Byte 6: Temperature
    // Byte 7: Reserved
    
    encoder_raw_ = static_cast<uint16_t>((frame.data[0] << 8) | frame.data[1]);
    velocity_raw_ = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
    current_raw_ = static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
    temperature_ = frame.data[6];
}

void RmActuator::updateMultiTurnPosition() {
    if (seq_count_ > 0) {  // Not first reading
        int16_t encoder_diff = encoder_raw_ - encoder_last_;
        
        // Detect encoder overflow/underflow
        if (encoder_diff > 4096) {
            // Encoder wrapped from 8191 to 0
            turn_count_--;
        } else if (encoder_diff < -4096) {
            // Encoder wrapped from 0 to 8191
            turn_count_++;
        }
    }
    
    encoder_last_ = encoder_raw_;
}

can_frame RmActuator::generateCommandFrame() {
    can_frame frame{};
    frame.can_id = tx_id_;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);
    
    // Convert effort command to motor current command
    ActuatorCoefficients coeff = getCoefficientsFor();
    int16_t effort_command = static_cast<int16_t>(std::clamp(
        cmd_effort_ * coeff.effort2act,
        static_cast<double>(-max_effort_),
        static_cast<double>(max_effort_)
    ));
    
    // Pack command into frame at motor-specific position
    // Each motor occupies 2 bytes in the frame
    uint8_t byte_offset = motor_index_ * 2;
    frame.data[byte_offset] = (effort_command >> 8) & 0xFF;
    frame.data[byte_offset + 1] = effort_command & 0xFF;
    
    // Debug output (throttled)
    if (seq_count_ % 1000 == 0) {
        RCLCPP_DEBUG(logger_, "RM %s: sending effort command %d (%.3f N⋅m)", 
                    name_.c_str(), effort_command, cmd_effort_);
    }
    
    return frame;
}

void RmActuator::validateConfiguration(const YAML::Node& config) {
    // Check required parameters
    if (!config["rx_id"] && !config["id"]) {
        throw std::invalid_argument("Missing rx_id or id in configuration for " + name_);
    }
    
    // Validate motor type
    if (model_.empty()) {
        throw std::invalid_argument("Motor type not specified for " + name_);
    }
    
    // Set defaults based on motor type
    if (model_.find("RM") == std::string::npos) {
        RCLCPP_WARN(logger_, "Unknown motor type '%s' for %s, using RM2006 defaults", 
                   model_.c_str(), name_.c_str());
    }
}

} // namespace device 