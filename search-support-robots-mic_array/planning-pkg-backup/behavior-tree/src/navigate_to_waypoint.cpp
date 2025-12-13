#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class NavigateToWaypoint : public BT::SyncActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToWaypoint(const std::string& name,
                     const BT::NodeConfig& config,
                     const rclcpp::Node::SharedPtr& node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    RCLCPP_INFO(node_->get_logger(), "[NavigateToWaypoint] Node created");
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToWaypoint] Waiting for Nav2 action server...");

    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available!");
      return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->get_clock()->now();
    goal.pose.pose.position.x = 1.0;
    goal.pose.pose.position.y = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    auto future_goal = client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_, future_goal) != rclcpp::FutureReturnCode::SUCCESS)
      return BT::NodeStatus::FAILURE;

    auto goal_handle = future_goal.get();
    auto result_future = client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(node_, result_future);

    RCLCPP_INFO(node_->get_logger(), "[NavigateToWaypoint] Goal complete!");
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};
