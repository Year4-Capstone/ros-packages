#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class ApproachPerson : public BT::SyncActionNode {
public:
  ApproachPerson(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::cout << "[ApproachPerson] Approaching target...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    return BT::NodeStatus::SUCCESS;
  }
};
