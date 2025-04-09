#include "Life_cycle_control/Detection.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto avoidance_node = std::make_shared<Life_cycle_control
  ::Detection>();
  rclcpp::spin(avoidance_node);

  rclcpp::shutdown();

  return 0;
}