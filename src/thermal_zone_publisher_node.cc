#include "thermal_zone_publisher/thermal_zone_publisher_node.h"

ThermalZonePublisherNode::ThermalZonePublisherNode(const std::string & node_name)
: rclcpp::Node(node_name)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "default constructor executed");
}

ThermalZonePublisherNode::~ThermalZonePublisherNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "destructor executed");
}
