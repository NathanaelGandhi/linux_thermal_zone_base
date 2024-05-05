#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "thermal_zone_interfaces/msg/thermal_zone.hpp"

class ThermalZonePublisherNode : public rclcpp::Node
{
public:
  explicit ThermalZonePublisherNode(const std::string & node_name);
  ~ThermalZonePublisherNode();

protected:
private:
  std::vector<thermal_zone_interfaces::msg::ThermalZone> GetZoneMsgVector(void);
  thermal_zone_interfaces::msg::ThermalZone GetZoneMsg(std::string key, uint8_t zone_index);
  double GetZoneTemperature(std::string prefix);
  uint8_t CountMatchingDirectories(const std::string & pattern);
};
