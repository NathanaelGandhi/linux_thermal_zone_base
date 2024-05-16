#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "linux_thermal_zone_base/linux_thermal_zone_base_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LinuxThermalZoneBaseNode>("linux_thermal_zone_base_node"));
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(std::make_shared<ThermalZoneNode>("thermal_zone_node"));
  // exec.spin();

  rclcpp::shutdown();

  return 0;
}
