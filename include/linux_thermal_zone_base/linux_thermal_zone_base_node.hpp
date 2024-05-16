#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
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
  size_t linux_thermal_zone_pub_count_;
  rclcpp::TimerBase::SharedPtr timer_1s_;
  rclcpp::TimerBase::SharedPtr timer_10s_;
  std::vector<rclcpp::Publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZone>::SharedPtr>
    publishers_linux_thermal_zone_;
  rclcpp::Publisher<linux_thermal_zone_interfaces::msg::LinuxThermalZoneBaseNodeHk>::SharedPtr
    publisher_node_hk_;

  void timer_1s_callback(void);
  void timer_10s_callback(void);
  std::vector<linux_thermal_zone_interfaces::msg::LinuxThermalZone> GetZoneMsgVector(void);
  linux_thermal_zone_interfaces::msg::LinuxThermalZone GetZoneMsg(
    std::string key, uint8_t zone_index);
  double GetZoneTemperature(std::string prefix);
  uint8_t CountMatchingDirectories(const std::string & pattern);
};
