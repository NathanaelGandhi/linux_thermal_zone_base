#include "thermal_zone/thermal_zone_node.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

// PUBLIC FUNCTIONS

ThermalZoneNode::ThermalZoneNode(const std::string & node_name) : rclcpp::Node(node_name)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "default constructor executed");

  std::vector<thermal_zone_interfaces::msg::ThermalZone> msgs = GetZoneMsgVector();
}

ThermalZoneNode::~ThermalZoneNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "destructor executed");
}

// PROTECTED FUNCTIONS

// PRIVATE FUNCTIONS

uint8_t ThermalZoneNode::CountMatchingDirectories(const std::string & pattern)
{
  uint8_t count = 0;

  for (const auto & entry : std::filesystem::directory_iterator("/sys/class/thermal/")) {
    if (
      std::filesystem::is_directory(entry) &&
      entry.path().filename().string().find(pattern) != std::string::npos) {
      ++count;
    }
  }
  return count;
}

std::vector<thermal_zone_interfaces::msg::ThermalZone> ThermalZoneNode::GetZoneMsgVector(void)
{
  std::vector<thermal_zone_interfaces::msg::ThermalZone> msgs;
  const std::string key = "thermal_zone";
  uint8_t num_zones = CountMatchingDirectories(key);

  for (uint8_t zone_index = 0; zone_index < num_zones; ++zone_index) {
    msgs.push_back(GetZoneMsg(key, zone_index));
    RCLCPP_INFO_STREAM(
      this->get_logger(), msgs.at(zone_index).temperature.header.frame_id
                            << " temperature: " << msgs.at(zone_index).temperature.temperature
                            << "°C");
  }

  return msgs;
}

thermal_zone_interfaces::msg::ThermalZone ThermalZoneNode::GetZoneMsg(
  std::string key, uint8_t zone_index)
{
  std::string prefix = "/sys/class/thermal/";

  auto msg = thermal_zone_interfaces::msg::ThermalZone();

  std::string id = key + std::to_string(zone_index);
  std::string thermal_zone_dir = prefix + id;

  msg.header.set__stamp(now());
  // temperature
  msg.temperature.header.set__stamp(now());
  msg.temperature.header.set__frame_id(id);
  msg.temperature.temperature = GetZoneTemperature(thermal_zone_dir);

  return msg;
}

double ThermalZoneNode::GetZoneTemperature(std::string thermal_zone_dir)
{
  std::string filename = "/temp";
  std::string filepath = thermal_zone_dir + filename;
  std::ifstream file(filepath);
  double temperature;

  if (file.is_open()) {
    std::string line;
    std::getline(file, line);  // Read the first line (assuming it contains the temperature)
    file.close();

    // Convert string to integer and handle errors
    try {
      temperature = std::stoi(line) / 1000;  // Convert millidegree Celsius to degree Celsius
      RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        thermal_zone_dir << " temperature: " << std::to_string(temperature) << "°C");
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid temperature data in file: " << filename);
    } catch (const std::out_of_range & e) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Temperature value out of range in file: " << filename);
    }
  } else {
    std::cerr << "Failed to open file " << filename << std::endl;
  }

  return temperature;
}
