#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "thermal_zone_interfaces/msg/thermal_zone.hpp"

class ThermalZoneNode : public rclcpp::Node
{
public:
  explicit ThermalZoneNode(const std::string & node_name);
  ~ThermalZoneNode();

protected:
private:
  std::vector<thermal_zone_interfaces::msg::ThermalZone> GetZoneMsgVector(void);
  thermal_zone_interfaces::msg::ThermalZone GetZoneMsg(std::string key, uint8_t zone_index);
  double GetZoneTemperature(std::string prefix);
  uint8_t CountMatchingDirectories(const std::string & pattern);
};
