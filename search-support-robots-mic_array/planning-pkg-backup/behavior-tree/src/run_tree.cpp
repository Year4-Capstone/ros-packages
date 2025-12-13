#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <iostream>

// Keep your includes of .cpp files OR switch to headers later (see Option A vs B)
#include "check_system.cpp"
#include "navigate_to_waypoint.cpp"
#include "detect_person.cpp"
#include "approach_person.cpp"
#include "deliver_payload.cpp"
#include "replan_search_area.cpp"
#include "report_to_base.cpp"

int main(int argc, char **argv)
{
  std::cout << "[run_tree] Starting program..." << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "[run_tree] ROS2 initialized" << std::endl;

  auto shared_node = rclcpp::Node::make_shared("bt_shared_node");
  std::cout << "[run_tree] Shared node created" << std::endl;

  BT::BehaviorTreeFactory factory;

  // Register nodes that don’t need extra args the usual way:
  factory.registerNodeType<CheckSystemOK>("CheckSystemOK");
  factory.registerNodeType<DetectPerson>("DetectPerson");
  factory.registerNodeType<ApproachPerson>("ApproachPerson");
  factory.registerNodeType<DeliverPayload>("DeliverPayload");
  factory.registerNodeType<ReplanSearchArea>("ReplanSearchArea");
  factory.registerNodeType<ReportToBase>("ReportToBase");

  // For NavigateToWaypoint, inject the shared node via a NodeBuilder:
  {
    BT::NodeBuilder builder = [shared_node](const std::string& name, const BT::NodeConfig& cfg)
    {
      // ctor signature: (name, cfg, shared_node)
      return std::make_unique<NavigateToWaypoint>(name, cfg, shared_node);
    };
    factory.registerBuilder<NavigateToWaypoint>("NavigateToWaypoint", builder);
  }

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behavior-tree");
  std::string xml_file = pkgpath + "/behavior_trees/search_and_support.xml";
  std::cout << "[run_tree] XML: " << xml_file << std::endl;

  auto tree = factory.createTreeFromFile(xml_file);
  std::cout << "[run_tree] Tree created" << std::endl;

  BT::Groot2Publisher publisher(tree, 5555);
  std::cout << "[run_tree] Groot2 publisher on 5555" << std::endl;

  // (Optional) give the shared node an executor if you add subscriptions/timers later
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(shared_node);

  while (rclcpp::ok())
  {
    std::cout << "[run_tree] Ticking root..." << std::endl;
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // exec.spin_some(); // if you enable executor above
  }

  rclcpp::shutdown();
  return 0;
}
