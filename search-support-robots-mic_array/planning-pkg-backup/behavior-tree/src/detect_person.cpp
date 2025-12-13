#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class DetectPerson : public BT::SyncActionNode {
public:
  DetectPerson(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    bool detected = (rand() % 3 == 0); // simulate detection
    std::cout << "[DetectPerson] Person " 
              << (detected ? "DETECTED!" : "not found") << "\n";
    return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
