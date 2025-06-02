#include "nodeconfigparser.h"
#include <iostream>

NodeConfigParser::NodeConfigParser(const std::string &file_path) {
    try {
        root = YAML::LoadFile(file_path);
        valid = true;
    } catch (const YAML::Exception &e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
    }
}

bool NodeConfigParser::isValid() const {
    return valid;
}

SerialConfig NodeConfigParser::getSerialConfig() const {
    SerialConfig config;
    if (root["serial_config"]) {
        config.baud = root["serial_config"]["baud"].as<int>();
        config.port = root["serial_config"]["port"].as<std::string>();
    }
    return config;
}

DiffDriveConfig NodeConfigParser::getDiffDriveConfig() const {
    DiffDriveConfig config;
    if (root["diff_drive_config"]) {
        config.wheel_radius = root["diff_drive_config"]["wheel_radius"].as<double>();
        config.wheel_base = root["diff_drive_config"]["wheel_base"].as<double>();
    }
    return config;
}
