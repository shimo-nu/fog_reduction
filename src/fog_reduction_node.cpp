#include "fog_reduction/fog_reduction_core.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto fog_reduction = std::make_shared<FogReduction>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(fog_reduction);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
