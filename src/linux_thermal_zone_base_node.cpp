#include "linux_thermal_zone_base/linux_thermal_zone_base_node.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

// PUBLIC FUNCTIONS

LinuxThermalZoneBaseNode::LinuxThermalZoneBaseNode(const std::string & node_name)
: rclcpp::Node(node_name), linux_thermal_zone_pub_count_(0)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "default constructor executed");

  // ros parameters
  auto rate_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  rate_param_desc.description = "Frequency (rate in Hz) of type: double";
  this->declare_parameter("data_pub_rate_hz", 1.0, rate_param_desc);
  this->declare_parameter("hk_pub_rate_hz", 0.1, rate_param_desc);
  this->declare_parameter("data_acquisition_rate_hz", 0.5, rate_param_desc);
  params.data_pub_rate_hz_ = this->get_parameter("data_pub_rate_hz").as_double();
  params.hk_pub_rate_hz_ = this->get_parameter("hk_pub_rate_hz").as_double();
  params.data_acquisition_rate_hz_ = this->get_parameter("data_acquisition_rate_hz").as_double();

  // threads
  data_acquisition_thread_ =
    std::thread(std::bind(&LinuxThermalZoneBaseNode::data_acquisition_thread, this));

  // timers
  data_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / params.data_pub_rate_hz_)),
    std::bind(&LinuxThermalZoneBaseNode::data_pub_timer_callback, this));
  hk_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / params.hk_pub_rate_hz_)),
    std::bind(&LinuxThermalZoneBaseNode::hk_pub_timer_callback, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "timers created");

  uint8_t num_thermal_zones = CountMatchingDirectories("thermal_zone");

  // publishers
  for (uint8_t zone_index = 0; zone_index < num_thermal_zones; zone_index++) {
    std::string zone_string = "thermal_zone" + std::to_string(zone_index);
    publishers_linux_thermal_zone_.push_back(
      this->create_publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZone>(
        zone_string, 10));
  }
  publisher_node_hk_ =
    this->create_publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk>(
      "node_hk", 10);
  RCLCPP_INFO_STREAM(this->get_logger(), "publishers created");
}

LinuxThermalZoneBaseNode::~LinuxThermalZoneBaseNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "destructor executed");

  data_acquisition_thread_.join();
}

// PROTECTED FUNCTIONS

// PRIVATE FUNCTIONS

void LinuxThermalZoneBaseNode::data_acquisition_thread(void)
{
  rclcpp::Rate rate(params.data_acquisition_rate_hz_);
  while (rclcpp::ok()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "data_acquisition_thread executed on thread id: " << std::this_thread::get_id());
    auto msgs = GetZoneMsgVector();
    std::unique_lock<std::mutex> lock(linux_thermal_zone_msgs_mutex_);
    linux_thermal_zone_msgs_ = msgs;
    lock.unlock();  // unlock the mutex explicitly
    rate.sleep();   // sleep to maintain the loop rate
  }
}

void LinuxThermalZoneBaseNode::data_pub_timer_callback()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "data_pub_timer_callback executed");

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Publishing: " << linux_thermal_zone_msgs_.size() << " LinuxThermalZone messages");

  const std::lock_guard<std::mutex> lock(
    linux_thermal_zone_msgs_mutex_);  // lock until end of scope
  for (uint8_t index = 0; index < publishers_linux_thermal_zone_.size(); index++) {
    publishers_linux_thermal_zone_.at(index)->publish(linux_thermal_zone_msgs_.at(index));
    linux_thermal_zone_pub_count_++;
  }
}

void LinuxThermalZoneBaseNode::hk_pub_timer_callback()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "hk_pub_timer_callback executed");
  linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk message;
  message.set__linux_thermal_zone_publish_count(linux_thermal_zone_pub_count_);
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

  uint8_t num_thermal_zones = CountMatchingDirectories(key);

  for (uint8_t zone_index = 0; zone_index < num_thermal_zones; ++zone_index) {
    msgs.push_back(GetZoneMsg(key, zone_index));
    RCLCPP_DEBUG_STREAM(
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

  // header
  msg.header.set__stamp(now());
  msg.header.set__frame_id(id);
  // temperature
  msg.temperature.header.set__stamp(now());
  msg.temperature.header.set__frame_id(id);
  msg.temperature.temperature = GetZoneTemperature(thermal_zone_dir);
  // type
  msg.type = GetZoneString(thermal_zone_dir + "/type");

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

std::string LinuxThermalZoneBaseNode::GetZoneString(std::string filepath)
{
  std::ifstream file(filepath);
  std::string return_string;

  if (file.is_open()) {
    std::string line;
    std::getline(file, line);  // Read the first line (assuming it contains the value)
    file.close();

    // Copy string to return and handle errors
    try {
      return_string = line;
      RCLCPP_DEBUG_STREAM(this->get_logger(), filepath << " value: " << return_string);
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid type data in file: " << filepath);
    } catch (const std::out_of_range & e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Type value out of range in file: " << filepath);
    }
  } else {
    std::cerr << "Failed to open file " << filepath << std::endl;
  }

  return return_string;
}
