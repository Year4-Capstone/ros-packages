#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  std::cout << "[test] Initializing rclcpp..." << std::endl;
  rclcpp::init(argc, argv);

  std::cout << "[test] Creating node..." << std::endl;
  auto node = rclcpp::Node::make_shared("test_node");
  std::cout << "[test] Node created: " << node->get_fully_qualified_name() << std::endl;

  rclcpp::shutdown();
  std::cout << "[test] Shutdown complete." << std::endl;
  return 0;
}
