#include "config_reader.hpp"
#include <stdexcept>

ConfigReader::ConfigReader(const std::string& config_file)
    : config_file_(config_file)
    , robot_(1.0, 0.5, 0.4, Point_3(0, 0, 0))  // Default robot dimensions and position
{}

void ConfigReader::parse() {
    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        
        // Parse cameras
        if (config["cameras"]) {
            for (const auto& camera_node : config["cameras"]) {
                cameras_.push_back(parseCameraConfig(camera_node));
            }
        }
        
        // Parse robot configuration
        if (config["robot"]) {
            robot_ = parseRobotConfig(config["robot"]);
        }
        
        // Parse required overlap
        if (config["required_overlap"]) {
            required_overlap_ = config["required_overlap"].as<double>();
        }
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse config file: " + std::string(e.what()));
    }
}

Camera ConfigReader::parseCameraConfig(const YAML::Node& camera_node) {
    // Parse position
    auto pos = camera_node["position"];
    Point_3 position(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
    
    // Parse orientation (roll, pitch, yaw in radians)
    auto orientation = camera_node["orientation"];
    double roll = orientation["roll"].as<double>();
    double pitch = orientation["pitch"].as<double>();
    double yaw = orientation["yaw"].as<double>();
    
    // Parse camera parameters
    double hfov = camera_node["hfov"].as<double>();
    double vfov = camera_node["vfov"].as<double>();
    double near_plane = camera_node["near_plane"].as<double>();
    double far_plane = camera_node["far_plane"].as<double>();
    
    return Camera(position, roll, pitch, yaw, hfov, vfov, near_plane, far_plane);
}

Robot ConfigReader::parseRobotConfig(const YAML::Node& robot_node) {
    double length = robot_node["length"].as<double>();
    double width = robot_node["width"].as<double>();
    double height = robot_node["height"].as<double>();
    
    // Parse position if available, otherwise use origin
    Point_3 position(0, 0, 0);
    if (robot_node["position"]) {
        auto pos = robot_node["position"];
        position = Point_3(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
    }
    
    return Robot(length, width, height, position);
} 