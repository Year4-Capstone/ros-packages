#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class DeliverPayload : public BT::SyncActionNode {
public:
  DeliverPayload(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[DeliverPayload] Dropping payload.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return BT::NodeStatus::SUCCESS;
  }
};
