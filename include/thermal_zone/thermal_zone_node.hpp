#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "thermal_zone_interfaces/msg/thermal_zone.hpp"
#include "thermal_zone_interfaces/msg/thermal_zone_node_hk.hpp"

class ThermalZoneNode : public rclcpp::Node
{
public:
  explicit ThermalZoneNode(const std::string & node_name);
  ~ThermalZoneNode();

protected:
private:
  size_t thermal_pub_count_;
  rclcpp::TimerBase::SharedPtr timer_1s_;
  rclcpp::TimerBase::SharedPtr timer_10s_;
  rclcpp::Publisher<thermal_zone_interfaces::msg::ThermalZone>::SharedPtr publisher_thermal_;
  rclcpp::Publisher<thermal_zone_interfaces::msg::ThermalZoneNodeHk>::SharedPtr publisher_node_hk_;

  void timer_1s_callback(void);
  void timer_10s_callback(void);
  std::vector<thermal_zone_interfaces::msg::ThermalZone> GetZoneMsgVector(void);
  thermal_zone_interfaces::msg::ThermalZone GetZoneMsg(std::string key, uint8_t zone_index);
  double GetZoneTemperature(std::string prefix);
  uint8_t CountMatchingDirectories(const std::string & pattern);
};
