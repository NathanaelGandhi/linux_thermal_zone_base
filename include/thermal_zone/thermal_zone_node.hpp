#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

class ThermalZonePublisherNode : public rclcpp::Node
{
public:
  explicit ThermalZonePublisherNode(const std::string & node_name);
  ~ThermalZonePublisherNode();

protected:
private:
};
