#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "phidgets_test/action/homing_sequence.hpp"

class HomingClient : public rclcpp::Node
{
public:
    using HomingSequence = phidgets_test::action::HomingSequence;
    using GoalHandleHomingSequence = rclcpp_action::ClientGoalHandle<HomingSequence>;

    HomingClient() : Node("homing_client")
    {
        client_ = rclcpp_action::create_client<HomingSequence>(
            this, "homing_sequence");

        this->send_goal();
    }

private:
    rclcpp_action::Client<HomingSequence>::SharedPtr client_;

    void send_goal()
    {
        using namespace std::placeholders;

        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = HomingSequence::Goal();
        goal_msg.homing_speed = 0.1;  // 10 RPM as requested

        RCLCPP_INFO(get_logger(), "Sending homing goal");

        auto send_goal_options = rclcpp_action::Client<HomingSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&HomingClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&HomingClient::result_callback, this, _1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedback_callback(
        GoalHandleHomingSequence::SharedPtr,
        const std::shared_ptr<const HomingSequence::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Homing status: ";
        for (size_t i = 0; i < feedback->status.size(); ++i) {
            ss << "Leg " << i << ": ";
            switch (feedback->status[i]) {
                case 0: ss << "Not Started"; break;
                case 1: ss << "In Progress"; break;
                case 2: ss << "Completed"; break;
                case 3: ss << "Failed"; break;
            }
            ss << " (pos: " << feedback->positions[i] << "°) | ";
        }
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }

    void result_callback(const GoalHandleHomingSequence::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Homing succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Homing aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Homing canceled");
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                return;
        }

        std::stringstream ss;
        ss << "Final results: ";
        for (size_t i = 0; i < result.result->success.size(); ++i) {
            ss << "Leg " << i << ": " 
               << (result.result->success[i] ? "Success" : "Failed")
               << " (pos: " << result.result->positions[i] << "°) | ";
        }
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomingClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}