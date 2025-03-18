#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "camera.hpp"
#include "robot.hpp"

class ConfigReader {
public:
    ConfigReader(const std::string& config_file);

    // Parse the configuration file
    void parse();

    // Getters
    std::vector<Camera> getCameras() const { return cameras_; }
    Robot getRobot() const { return robot_; }
    double getRequiredOverlap() const { return required_overlap_; }

private:
    std::string config_file_;
    std::vector<Camera> cameras_;
    Robot robot_{1.0, 0.5, 0.4}; // Default robot dimensions
    double required_overlap_{0.2}; // Default 20% overlap

    // Helper functions
    Camera parseCameraConfig(const YAML::Node& camera_node);
    Robot parseRobotConfig(const YAML::Node& robot_node);
};

#endif // CONFIG_READER_HPP 