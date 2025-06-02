#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

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

    bool isValid() const;

    SerialConfig getSerialConfig() const;
    DiffDriveConfig getDiffDriveConfig() const;

private:
    YAML::Node root;
    bool valid = false;
};
