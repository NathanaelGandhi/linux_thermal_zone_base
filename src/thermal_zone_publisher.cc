#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "thermal_zone_publisher/thermal_zone_publisher_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(std::make_shared<ThermalZonePublisherNode>("thermal_zone_publisher"));
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
