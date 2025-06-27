#pragma once

#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

struct SerialConfig {
    int baud;
    std::string port;
};

struct DiffDriveConfig {
    double wheel_radius;
    double wheel_base;
};

class NodeConfigParser {
public:
    explicit NodeConfigParser(const std::string &file_path);
    explicit NodeConfigParser(std::shared_ptr<rclcpp::Node> node, const std::string &default_file_path = "");

    bool isValid() const;

    SerialConfig getSerialConfig() const;
    DiffDriveConfig getDiffDriveConfig() const;

private:
    YAML::Node root;
    bool valid = false;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger;
    
    void loadConfigFromFile(const std::string &file_path);
};
