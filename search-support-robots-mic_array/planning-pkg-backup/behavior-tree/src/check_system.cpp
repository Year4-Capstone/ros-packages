#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class CheckSystemOK : public BT::SyncActionNode
{
public:
  CheckSystemOK(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("CheckSystemOK"), "System OK!");
    return BT::NodeStatus::SUCCESS;
  }
};
