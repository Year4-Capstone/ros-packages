#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
#include <chrono>
#include <iostream>

std::shared_ptr<rclcpp::Node> g_node;

class RemoteModeActive : public BT::ConditionNode {
public:
  RemoteModeActive(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    // TODO: Subscribe to /remote_mode later
    bool remote_mode = false; // TODO: Use this later
    return remote_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class AutonomousModeActive : public BT::ConditionNode {
public:
  AutonomousModeActive(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::ConditionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    bool remote_mode = false;
    return !remote_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class EnableTeleop : public BT::StatefulActionNode {
public:
  EnableTeleop(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::StatefulActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    RCLCPP_INFO(g_node->get_logger(), "[BT] Teleop enabled");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }

  void onHalted() override {
    RCLCPP_INFO(g_node->get_logger(), "[BT] Teleop halted");
  }
};

class MoveForward : public BT::StatefulActionNode {
public:
  MoveForward(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Time start_time_;

  BT::NodeStatus onStart() override {
    pub_ = g_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    start_time_ = g_node->now();
    RCLCPP_INFO(g_node->get_logger(), "[BT] MoveForward start");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.2;
    pub_->publish(cmd);

    if ((g_node->now() - start_time_).seconds() > 3.0) {
      RCLCPP_INFO(g_node->get_logger(), "[BT] MoveForward done");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    geometry_msgs::msg::Twist stop;
    pub_->publish(stop);
    RCLCPP_INFO(g_node->get_logger(), "[BT] MoveForward halted");
  }
};

class StopMotion : public BT::SyncActionNode {
public:
  StopMotion(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::SyncActionNode(name, cfg) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    auto pub = g_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist stop;
    pub->publish(stop);
    RCLCPP_INFO(g_node->get_logger(), "[BT] StopMotion");
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv)
{
  std::cout << "[search_support_bt_runner] Starting..." << std::endl;
  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("bt_shared_node");
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<RemoteModeActive>("RemoteModeActive");
  factory.registerNodeType<AutonomousModeActive>("AutonomousModeActive");
  factory.registerNodeType<EnableTeleop>("EnableTeleop");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<StopMotion>("StopMotion");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("behavior-tree");
  std::string xml_file = pkgpath + "/search_and_support.xml";
  std::cout << "[search_support_bt_runner] XML: " << xml_file << std::endl;

  auto tree = factory.createTreeFromFile(xml_file);
  std::cout << "[search_support_bt_runner] Tree created" << std::endl;

  BT::Groot2Publisher groot_pub(tree, 5555);
  std::cout << "[search_support_bt_runner] Groot2 publisher started (port 5555)" << std::endl;

  BT::printTreeRecursively(tree.rootNode());
  size_t tick_count = 0;

  while (rclcpp::ok())
  {
    std::cout << "\n==============================" << std::endl;
    std::cout << "[Tick #" << ++tick_count << "] Root tick start" << std::endl;

    BT::NodeStatus result = tree.tickOnce();

    std::cout << "[Tick #" << tick_count << "] Root returned: "
              << BT::toStr(result, true) << std::endl;

    const auto &nodes = tree.subtrees.front()->nodes;
    for (const auto &node : nodes)
    {
      std::cout << "  [Node] " << node->registrationName()
                << " (" << node->name() << ") -> "
                << BT::toStr(node->status(), true) << std::endl;
    }

    std::cout << "==============================\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(g_node);
  }

  rclcpp::shutdown();
  return 0;
}