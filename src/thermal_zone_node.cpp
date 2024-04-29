#include "thermal_zone/thermal_zone_node.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "thermal_zone_interfaces/msg/thermal_zone.hpp"

namespace fs = std::filesystem;

static int CountMatchingDirectories(const std::string & pattern)
{
  int count = 0;
  for (const auto & entry : fs::directory_iterator("/sys/class/thermal/")) {
    if (
      fs::is_directory(entry) &&
      entry.path().filename().string().find(pattern) != std::string::npos) {
      ++count;
    }
  }
  return count;
}

ThermalZonePublisherNode::ThermalZonePublisherNode(const std::string & node_name)
: rclcpp::Node(node_name)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "default constructor executed");

  std::string key = "thermal_zone";
  std::string prefix = "/sys/class/thermal/";
  std::string suffix = "/temp";

  int num_zones = CountMatchingDirectories(key);

  for (int i = 0; i < num_zones; ++i) {
    std::string id = key + std::to_string(i);

    auto msg = std::make_shared<thermal_zone_interfaces::msg::ThermalZone>();

    std::string filename = prefix + id + suffix;
    std::ifstream file(filename);

    if (file.is_open()) {
      std::string line;
      std::getline(file, line);  // Read the first line (assuming it contains the temperature)
      file.close();

      // Convert string to integer and handle errors
      try {
        int temp = std::stoi(line) / 1000;  // Convert millidegree Celsius to degree Celsius
        std::cout << "Zone " << i << " temperature: " << temp << "°C" << std::endl;
      } catch (const std::invalid_argument & e) {
        std::cerr << "Invalid temperature data in file " << filename << std::endl;
      } catch (const std::out_of_range & e) {
        std::cerr << "Temperature value out of range in file " << filename << std::endl;
      }
    } else {
      std::cerr << "Failed to open file " << filename << std::endl;
    }
  }
}

ThermalZonePublisherNode::~ThermalZonePublisherNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "destructor executed");
}
