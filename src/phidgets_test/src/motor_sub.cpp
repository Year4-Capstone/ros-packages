#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "phidgets_test/phidgets_bldc_driver.hpp"
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

const int SERIAL_NUMBER_1 = 527103;
const int SERIAL_NUMBER_2 = 527164;
const int PORT_0 = 2; // back left
const int PORT_1 = 3; // back right
const int PORT_2 = 2; // front left
const int PORT_3 = 3; // front right

class MotorSub : public rclcpp::Node
{
  public:
    MotorSub()
        : Node("robot_driver"),
          // Initialize motors in the member initializer list
          motor_bl_(SERIAL_NUMBER_1, PORT_0),
          motor_br_(SERIAL_NUMBER_1, PORT_1),
          motor_fl_(SERIAL_NUMBER_2, PORT_2),
          motor_fr_(SERIAL_NUMBER_2, PORT_3)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Robot Driver node...");

        // Initialize all motors
        motor_bl_.init();
        motor_br_.init();
        motor_fl_.init();
        motor_fr_.init();
        
        RCLCPP_INFO(this->get_logger(), "Motors initialized.");

        // Create the subscriber to /cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorSub::cmd_vel_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /cmd_vel.");
    }
    ~MotorSub()
    {
        // Ensure motors are stopped and cleaned up
        motor_bl_.setVelocityDuty(0.0);
        motor_br_.setVelocityDuty(0.0);
        motor_fl_.setVelocityDuty(0.0);
        motor_fr_.setVelocityDuty(0.0);
        
        motor_bl_.cleanup();
        motor_br_.cleanup();
        motor_fl_.cleanup();
        motor_fr_.cleanup();
        RCLCPP_INFO(this->get_logger(), "Motors stopped and cleaned up.");
    }

  private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 1. Get velocities from the Twist message
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // 2. Calculate left and right side speeds (kinematic mixing)
        double left_speed = linear - angular;
        double right_speed = linear + angular;

        // 3. Normalize speeds if they exceed 1.0
        double max_speed = std::max(std::abs(left_speed), std::abs(right_speed));
        if (max_speed > 1.0) {
            left_speed /= max_speed;
            right_speed /= max_speed;
        }

        // 4. CLAMP the values to be strictly within [-1.0, 1.0]
        double final_left = std::clamp(left_speed, -1.0, 1.0);
        double final_right = std::clamp(right_speed, -1.0, 1.0);    
        // 5. Send the clamped commands to the motors
        // This mapping matches your test script logic (left vs right)
        
        // Left Side
        motor_fl_.setVelocityDuty(final_left); // Front Left
        motor_bl_.setVelocityDuty(final_left); // Back Left

        // Right Side
        motor_fr_.setVelocityDuty(-final_right); // Front Right
        motor_br_.setVelocityDuty(-final_right); // Back Right
    }

    // Member variables for motors
    PhidgetDriveMotorController motor_bl_; // motor0
    PhidgetDriveMotorController motor_br_; // motor1
    PhidgetDriveMotorController motor_fl_; // motor2
    PhidgetDriveMotorController motor_fr_; // motor3

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSub>());
  rclcpp::shutdown();
  return 0;
}