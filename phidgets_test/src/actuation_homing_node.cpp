#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "phidgets_test/phidgets_motor_driver.hpp"
#include "phidgets_test/phidgets_limit_switch.hpp"
#include "phidgets_test/action/homing_sequence.hpp"

#include <memory>
#include <array>
#include <thread>
#include <functional>
#include <atomic>

class ActuatorHomingNode : public rclcpp::Node
{
public:
    using HomingSequence = phidgets_test::action::HomingSequence;
    using GoalHandleHomingSequence = rclcpp_action::ServerGoalHandle<HomingSequence>;

    ActuatorHomingNode() : Node("actuation_homing_node")
    {
        // Constants from test files
        const int SERIAL_NUMBER_1 = 527103;
        const int SERIAL_NUMBER_2 = 527164;
        //const int MOTOR_PORT_0 = 0;
        //const int MOTOR_PORT_1 = 1;
        const int MOTOR_PORT_2 = 2; // front left
        const int MOTOR_PORT_3 = 3; // front right
        const int LIMIT_PORT_4 = 4;
        const int LIMIT_PORT_5 = 5;

        // Initialize motors
        motors_[0] = std::make_unique<PhidgetMotorController>(SERIAL_NUMBER_1, MOTOR_PORT_2, MotorType::Drive, +1);
        motors_[1] = std::make_unique<PhidgetMotorController>(SERIAL_NUMBER_1, MOTOR_PORT_3, MotorType::Drive, +1);
        motors_[2] = std::make_unique<PhidgetMotorController>(SERIAL_NUMBER_2, MOTOR_PORT_2, MotorType::Drive, +1);
        motors_[3] = std::make_unique<PhidgetMotorController>(SERIAL_NUMBER_2, MOTOR_PORT_3, MotorType::Drive, +1);

        // Initialize limit switches
        limit_switches_[0] = std::make_unique<PhidgetLimitSwitch>(SERIAL_NUMBER_1, LIMIT_PORT_4);
        limit_switches_[1] = std::make_unique<PhidgetLimitSwitch>(SERIAL_NUMBER_1, LIMIT_PORT_5);
        limit_switches_[2] = std::make_unique<PhidgetLimitSwitch>(SERIAL_NUMBER_2, LIMIT_PORT_4);
        limit_switches_[3] = std::make_unique<PhidgetLimitSwitch>(SERIAL_NUMBER_2, LIMIT_PORT_5);
        
        // Init triggered flags + callbacks
        for (int i = 0; i < 4; ++i) {
            limit_triggered_[i].store(false);

            limit_switches_[i]->setCallback([this, i](int state) {
                // 1 = PRESSED, 0 = RELEASED
                limit_triggered_[i].store(state == 1);
            });
        }

        // Initialize all devices
        for (int i = 0; i < 4; ++i) {
            motors_[i]->init();
            limit_switches_[i]->init();
        }

        // Create action server
        this->action_server_ = rclcpp_action::create_server<HomingSequence>(
            this,
            "homing_sequence",
            std::bind(&ActuatorHomingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActuatorHomingNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActuatorHomingNode::handle_accepted, this, std::placeholders::_1));
    }

    ~ActuatorHomingNode()
    {
        // Cleanup all devices
        for (int i = 0; i < 4; ++i) {
            if (motors_[i]) motors_[i]->cleanup();
            if (limit_switches_[i]) limit_switches_[i]->cleanup();
        }
    }

private:
    std::array<std::unique_ptr<PhidgetMotorController>, 4> motors_;
    std::array<std::unique_ptr<PhidgetLimitSwitch>, 4> limit_switches_;
    rclcpp_action::Server<HomingSequence>::SharedPtr action_server_;
    std::array<std::atomic<bool>, 4> limit_triggered_;
    const double BACKOFF_ANGLE = 10.0; // degrees

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const HomingSequence::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received homing request with speed: %f RPM", goal->homing_speed);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleHomingSequence>)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel homing");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleHomingSequence> goal_handle)
    {
        std::thread{std::bind(&ActuatorHomingNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleHomingSequence> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<HomingSequence::Feedback>();
        auto result = std::make_shared<HomingSequence::Result>();
        
        // Initialize feedback and result arrays
        feedback->status.resize(4, 0);    // 0 = not started
        feedback->positions.resize(4, 0.0);
        result->success.resize(4, false);
        result->positions.resize(4, 0.0);

        // Start homing sequence for all motors in parallel
        std::array<std::thread, 4> homing_threads;
        for (size_t i = 0; i < 4; ++i) {
            homing_threads[i] = std::thread([this, i, goal, goal_handle, feedback]() {
                this->home_single_motor(i, goal->homing_speed, goal_handle, feedback);
            });
        }

        // Wait for all threads to complete
        for (auto& thread : homing_threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        // Prepare final result
        for (size_t i = 0; i < 4; ++i) {
            result->success[i] = (feedback->status[i] == 2);  // 2 = completed
            result->positions[i] = motors_[i]->getPositionDegs();
        }

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
    }

    void home_single_motor(size_t index, double homing_speed, 
                          const std::shared_ptr<GoalHandleHomingSequence>& goal_handle,
                          const std::shared_ptr<HomingSequence::Feedback>& feedback)
    {
        if (goal_handle->is_canceling()) return;

        // Start homing
        feedback->status[index] = 1;  // 1 = in progress
        goal_handle->publish_feedback(feedback);

        limit_triggered_[index].store(false);

        // Move toward limit switch (negative direction)
        motors_[index]->setVelocityDuty(-homing_speed);

        // Wait for limit switch to be triggered
        while (!limit_switches_[index]->read() && !goal_handle->is_canceling()) {
            feedback->positions[index] = motors_[index]->getPositionDegs();
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Stop motor
        motors_[index]->setVelocityDuty(0.0);

        if (goal_handle->is_canceling()) return;


        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (!limit_triggered_[index].load()) {
            feedback->status[index] = 3; // failed
            goal_handle->publish_feedback(feedback);
            return;
        }

        // Back off by BACKOFF_ANGLE degrees
        limit_triggered_[index].store(false);
        double start_pos = motors_[index]->getPositionDegs();
        double target_pos = start_pos + BACKOFF_ANGLE;
        
        // Move in positive direction
        motors_[index]->setVelocityDuty(homing_speed);
        
        // Wait until we've moved BACKOFF_ANGLE degrees
        while (motors_[index]->getPositionDegs() < target_pos && !goal_handle->is_canceling()) {
            feedback->positions[index] = motors_[index]->getPositionDegs();
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Stop motor
        motors_[index]->setVelocityDuty(0.0);

        if (goal_handle->is_canceling()) return;

        // Reset position to 0 after homing
        motors_[index]->resetPosition();

        // Mark as completed
        feedback->status[index] = 2;  // 2 = completed
        goal_handle->publish_feedback(feedback);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActuatorHomingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}