#include "linux_thermal_zone_base/linux_thermal_zone_base_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

// PUBLIC FUNCTIONS

LinuxThermalZoneBaseNode::LinuxThermalZoneBaseNode(const std::string & node_name)
: rclcpp::Node(node_name), thermal_pub_count_(0)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "default constructor executed");

  timer_1s_ =
    this->create_wall_timer(1s, std::bind(&LinuxThermalZoneBaseNode::timer_1s_callback, this));
  timer_10s_ =
    this->create_wall_timer(10s, std::bind(&LinuxThermalZoneBaseNode::timer_10s_callback, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "timers created");

  publisher_thermal_ =
    this->create_publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZone>("thermal", 10);
  publisher_node_hk_ =
    this->create_publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk>(
      "node_hk", 10);
  RCLCPP_INFO_STREAM(this->get_logger(), "publishers created");
}

LinuxThermalZoneBaseNode::~LinuxThermalZoneBaseNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "destructor executed");
}

// PROTECTED FUNCTIONS

// PRIVATE FUNCTIONS

void LinuxThermalZoneBaseNode::timer_1s_callback()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "timer_1s_callback executed");
  std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone> msgs = GetZoneMsgVector();

  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << msgs.size() << " ThermalZone messages");
  for (linux_thermal_zone_interfaces::msg::LinuxThermalZone message : msgs) {
    publisher_thermal_->publish(message);
    thermal_pub_count_++;
  }
}

void LinuxThermalZoneBaseNode::timer_10s_callback()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "timer_10s_callback executed");
  linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk message;
  message.set__linux_thermal_zone_publish_count(thermal_pub_count_);
  publisher_node_hk_->publish(message);
}

uint8_t LinuxThermalZoneBaseNode::CountMatchingDirectories(const std::string & pattern)
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

std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone>
LinuxThermalZoneBaseNode::GetZoneMsgVector(void)
{
  std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone> msgs;
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

linux_thermal_zone_interfaces::msg::LinuxThermalZone LinuxThermalZoneBaseNode::GetZoneMsg(
  std::string key, uint8_t zone_index)
{
  std::string prefix = "/sys/class/thermal/";

  auto msg = linux_thermal_zone_interfaces::msg::LinuxThermalZone();

  std::string id = key + std::to_string(zone_index);
  std::string thermal_zone_dir = prefix + id;

  msg.header.set__stamp(now());
  // temperature
  msg.temperature.header.set__stamp(now());
  msg.temperature.header.set__frame_id(id);
  msg.temperature.temperature = GetZoneTemperature(thermal_zone_dir);

  return msg;
}

double LinuxThermalZoneBaseNode::GetZoneTemperature(std::string thermal_zone_dir)
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
