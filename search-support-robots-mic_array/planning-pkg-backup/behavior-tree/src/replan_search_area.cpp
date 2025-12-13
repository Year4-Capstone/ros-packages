#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class ReplanSearchArea : public BT::SyncActionNode {
public:
  ReplanSearchArea(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[ReplanSearchArea] No detections. Replanning...\n";
    return BT::NodeStatus::SUCCESS;
  }
};
