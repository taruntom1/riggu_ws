#include "nodeconfigparser.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

NodeConfigParser::NodeConfigParser(const std::string &file_path) 
    : node_(nullptr), logger(rclcpp::get_logger("NodeConfigParser")) {
    loadConfigFromFile(file_path);
}

NodeConfigParser::NodeConfigParser(std::shared_ptr<rclcpp::Node> node, const std::string &default_file_path)
    : node_(node), logger(node ? node->get_logger() : rclcpp::get_logger("NodeConfigParser")) {
    
    std::string config_path;
    
    if (node_) {
        // Determine default path if not provided
        std::string default_path = default_file_path;
        if (default_path.empty()) {
            std::string package_path = ament_index_cpp::get_package_share_directory("drivelink_interface");
            default_path = package_path + "/config/node_config.yaml";
        }
        
        try {
            // Declare parameter with default value
            node_->declare_parameter<std::string>("node_config_path", default_path);
            
            // Get the parameter value
            config_path = node_->get_parameter("node_config_path").as_string();
            
            RCLCPP_INFO(logger, "Reading node configuration YAML from parameter 'node_config_path': %s", config_path.c_str());
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
            // Parameter already declared, just get its value
            config_path = node_->get_parameter("node_config_path").as_string();
            
            RCLCPP_DEBUG(logger, "Using existing parameter 'node_config_path': %s", config_path.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "Error handling parameter 'node_config_path': %s. Using default path.", e.what());
            config_path = default_path;
        }
    } else {
        // Fallback when no node is provided
        config_path = default_file_path.empty() ? 
            ament_index_cpp::get_package_share_directory("drivelink_interface") + "/config/node_config.yaml" :
            default_file_path;
        
        RCLCPP_WARN(logger, "No ROS2 node provided, using default path: %s", config_path.c_str());
    }
    
    loadConfigFromFile(config_path);
}

void NodeConfigParser::loadConfigFromFile(const std::string &file_path) {
    try {
        root = YAML::LoadFile(file_path);
        valid = true;
        RCLCPP_INFO(logger, "Successfully loaded configuration from: %s", file_path.c_str());
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(logger, "YAML parsing error for file '%s': %s", file_path.c_str(), e.what());
        valid = false;
    }
}

bool NodeConfigParser::isValid() const {
    return valid;
}

SerialConfig NodeConfigParser::getSerialConfig() const {
    SerialConfig config;
    if (valid && root["serial_config"]) {
        config.baud = root["serial_config"]["baud"].as<int>();
        config.port = root["serial_config"]["port"].as<std::string>();
        
        if (node_) {
            RCLCPP_DEBUG(logger, "Loaded serial config: port=%s, baud=%d", config.port.c_str(), config.baud);
        }
    } else {
        if (node_) {
            RCLCPP_WARN(logger, "Serial config not found in YAML or YAML is invalid, using defaults");
        }
    }
    return config;
}

DiffDriveConfig NodeConfigParser::getDiffDriveConfig() const {
    DiffDriveConfig config;
    if (valid && root["diff_drive_config"]) {
        config.wheel_radius = root["diff_drive_config"]["wheel_radius"].as<double>();
        config.wheel_base = root["diff_drive_config"]["wheel_base"].as<double>();
        
        if (node_) {
            RCLCPP_DEBUG(logger, "Loaded diff drive config: wheel_radius=%f, wheel_base=%f", 
                        config.wheel_radius, config.wheel_base);
        }
    } else {
        if (node_) {
            RCLCPP_WARN(logger, "Diff drive config not found in YAML or YAML is invalid, using defaults");
        }
    }
    return config;
}
