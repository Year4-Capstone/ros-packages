#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
    {
    auto msg = getInput<std::string>("message").value_or("Hello, world!");
    std::cout << "SaySomething node says: " << msg << std::endl;

    // simulate work for half a second so you can see the RUNNING color
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return BT::NodeStatus::SUCCESS;
    }
};
