//
// CAN Manager implementation for ROS2
//

#include "actuator_interface/can_manager/can_manager.h"
#include "actuator_interface/can_manager/can_interface/socketcan.h"
#include <fstream>
#include <chrono>

namespace device {

CanManager::CanManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , running_(false)
    , config_loaded_(false)
    , logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "Initializing CAN Manager");
}

CanManager::~CanManager() {
    close();
}

bool CanManager::init() {
    RCLCPP_INFO(logger_, "Initializing CAN Manager");
    
    if (!loadBusConfig()) {
        RCLCPP_ERROR(logger_, "Failed to load CAN bus configuration");
        return false;
    }

    if (!loadDeviceConfig()) {
        RCLCPP_ERROR(logger_, "Failed to load CAN device configuration");
        return false;
    }

    RCLCPP_INFO(logger_, "CAN Manager initialization completed successfully");
    return true;
}

bool CanManager::loadBusConfig() {
    // Bus configuration is now loaded from YAML file in loadConfigFromFile
    // This method is kept for compatibility but will use default if no config file
    
    if (!config_loaded_) {
        RCLCPP_WARN(logger_, "No config file loaded, using default CAN bus: can0");
        return addCanBus("can0", 95);
    }
    
    return true;  // Configuration already loaded from YAML
}

bool CanManager::loadDeviceConfig() {
    // Device configuration is now loaded from YAML file in loadConfigFromFile
    // This method is kept for compatibility
    
    if (!config_loaded_) {
        RCLCPP_ERROR(logger_, "No config file loaded, cannot load device configuration");
        return false;
    }
    
    return true;  // Configuration already loaded from YAML
}

bool CanManager::loadConfigFromFile(const std::string& config_file) {
    try {
        RCLCPP_INFO(logger_, "Loading configuration from file: %s", config_file.c_str());
        
        YAML::Node root = YAML::LoadFile(config_file);
        
        if (!root["actuator_interface_hw"]) {
            RCLCPP_ERROR(logger_, "No actuator_interface_hw section found in config file");
            return false;
        }
        
        YAML::Node hw_config = root["actuator_interface_hw"];
        
        // Load bus configuration
        if (hw_config["buses"]) {
            for (const auto& bus_node : hw_config["buses"]) {
                std::string bus_name = bus_node["name"].as<std::string>();
                int priority = bus_node["priority"] ? bus_node["priority"].as<int>() : 95;
                
                if (!addCanBus(bus_name, priority)) {
                    RCLCPP_ERROR(logger_, "Failed to add CAN bus: %s", bus_name.c_str());
                    return false;
                }
                RCLCPP_INFO(logger_, "Added CAN bus: %s with priority %d", bus_name.c_str(), priority);
            }
        }
        
        // Load device configuration
        if (hw_config["devices"]) {
            for (const auto& device_node : hw_config["devices"]) {
                std::string name = device_node["name"].as<std::string>();
                std::string bus = device_node["bus"].as<std::string>();
                int id = device_node["id"].as<int>();
                std::string model = device_node["model"].as<std::string>();
                
                YAML::Node config = device_node["config"];
                
                if (!addDevice(name, bus, id, model, config)) {
                    RCLCPP_ERROR(logger_, "Failed to add device: %s", name.c_str());
                    return false;
                }
                RCLCPP_INFO(logger_, "Added device: %s", name.c_str());
            }
        }
        
        config_loaded_ = true;
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading config file: %s", e.what());
        return false;
    }
}

bool CanManager::addCanBus(const std::string& bus_name, int thread_priority) {
    try {
        auto socketcan = new can_interface::SocketCAN(bus_name, thread_priority);
        
        if (!socketcan->open()) {
            RCLCPP_ERROR(logger_, "Failed to open CAN bus: %s", bus_name.c_str());
            delete socketcan;
            return false;
        }
        
        can_buses_.push_back(socketcan);
        RCLCPP_INFO(logger_, "Successfully added CAN bus: %s", bus_name.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception adding CAN bus %s: %s", bus_name.c_str(), e.what());
        return false;
    }
}

bool CanManager::addDevice(const std::string& name, const std::string& bus, int id, 
                          const std::string& model, const YAML::Node& config) {
    try {
        std::shared_ptr<CanDevice> device;
        
        // Create device based on model type - extensible for any RM motor
        if (model.find("RM") != std::string::npos) {
            device = std::make_shared<RmActuator>(name, bus, id, model, config);
            
            // Add to actuator lists
            auto actuator = std::dynamic_pointer_cast<CanActuator>(device);
            if (actuator) {
                actuator_devices_[name] = actuator;
                actuator_names_.push_back(name);
            }
        } else {
            RCLCPP_ERROR(logger_, "Unsupported device model: %s. Currently supported: RM2006, RM3508, etc.", model.c_str());
            return false;
        }
        
        if (!device) {
            RCLCPP_ERROR(logger_, "Failed to create device: %s", name.c_str());
            return false;
        }
        
        // Add to general device list
        devices_[name] = device;
        
        // Add to bus device mapping
        bus_devices_[bus][id] = device;
        
        RCLCPP_INFO(logger_, "Successfully added device: %s (model: %s)", name.c_str(), model.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception adding device %s: %s", name.c_str(), e.what());
        return false;
    }
}

bool CanManager::start() {
    if (running_) {
        RCLCPP_WARN(logger_, "CAN Manager already running");
        return true;
    }
    
    try {
        // Start all devices
        for (auto& [name, device] : devices_) {
            device->start();
            RCLCPP_INFO(logger_, "Started device: %s", name.c_str());
        }
        
        running_ = true;
        RCLCPP_INFO(logger_, "CAN Manager started successfully");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception starting CAN Manager: %s", e.what());
        return false;
    }
}

void CanManager::close() {
    if (!running_) {
        return;
    }
    
    try {
        running_ = false;
        
        // Stop all devices
        for (auto& [name, device] : devices_) {
            device->close();
            RCLCPP_INFO(logger_, "Stopped device: %s", name.c_str());
        }
        
        // Close all CAN buses
        for (auto* bus : can_buses_) {
            bus->close();
            delete bus;
        }
        can_buses_.clear();
        
        RCLCPP_INFO(logger_, "CAN Manager closed");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception closing CAN Manager: %s", e.what());
    }
}

void CanManager::read() {
    if (!running_) {
        return;
    }
    
    try {
        // Read from all CAN buses
        for (auto* bus : can_buses_) {
            std::vector<can_interface::CanFrameStamp> buffer;
            if (bus->readBuffer(buffer, 100)) {
                // Distribute frames to appropriate devices
                for (const auto& frameStamp : buffer) {
                    // Find device by CAN ID
                    for (auto& [name, device] : devices_) {
                        // This is a simplified approach - in reality, you'd need
                        // more sophisticated CAN ID to device mapping
                        device->readBuffer(buffer);
                    }
                }
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception in CAN read: %s", e.what());
    }
}

void CanManager::write() {
    if (!running_) {
        return;
    }
    
    try {
        // Collect frames from all devices and write to appropriate buses
        std::unordered_map<std::string, std::vector<can_frame>> bus_frames;
        
        for (auto& [name, device] : devices_) {
            can_frame frame = device->write();
            if (frame.can_dlc > 0) {  // Valid frame
                bus_frames[device->getBus()].push_back(frame);
            }
        }
        
        // Write frames to buses
        for (auto* bus : can_buses_) {
            const std::string& bus_name = bus->getDeviceName();
            if (bus_frames.find(bus_name) != bus_frames.end()) {
                for (const auto& frame : bus_frames[bus_name]) {
                    bus->write(frame);
                }
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception in CAN write: %s", e.what());
    }
}

} // namespace device 