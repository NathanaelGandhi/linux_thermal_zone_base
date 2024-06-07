#pragma once

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "linux_thermal_zone_interfaces/msg/linux_thermal_zone.hpp"
#include "linux_thermal_zone_interfaces/msg/linux_thermal_zone_base_node_hk.hpp"

class LinuxThermalZoneBaseNode : public rclcpp::Node
{
public:
  explicit LinuxThermalZoneBaseNode(const std::string & node_name);
  ~LinuxThermalZoneBaseNode();

protected:
private:
  struct Params
  {
    double data_pub_rate_hz_;
    double hk_pub_rate_hz_;
    double data_acquisition_rate_hz_;
  } params;

  std::thread data_acquisition_thread_;
  std::mutex linux_thermal_zone_msgs_mutex_;
  std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone> linux_thermal_zone_msgs_;
  std::atomic<size_t> linux_thermal_zone_pub_count_;
  rclcpp::TimerBase::SharedPtr data_pub_timer_;
  rclcpp::TimerBase::SharedPtr hk_pub_timer_;
  std::vector<rclcpp::Publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZone>::SharedPtr>
    publishers_linux_thermal_zone_;
  rclcpp::Publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk>::SharedPtr
    publisher_node_hk_;

  void data_pub_timer_callback(void);
  void hk_pub_timer_callback(void);
  void data_acquisition_thread(void);
  std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone> GetZoneMsgVector(void);
  linux_thermal_zone_interfaces::msg::LinuxThermalZone GetZoneMsg(
    std::string key, uint8_t zone_index);
  double GetZoneTemperature(std::string thermal_zone_dir);
  std::string GetZoneString(std::string filepath);
  uint8_t CountMatchingDirectories(const std::string & pattern);
};
