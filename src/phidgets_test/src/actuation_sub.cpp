#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "phidgets_test/phidgets_actuaction_driver.hpp"
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

//test with drive motors than change
const int SERIAL_NUMBER_1 = 527103;
const int SERIAL_NUMBER_2 = 527164;
const int PORT_0 = 0; // back left
const int PORT_1 = 1; // back right
const int PORT_2 = 0; // front left
const int PORT_3 = 1; // front right

const double LEG_MOVE_SPEED = 0.4;//change
const int BUTTON_LEGS_UP = 0;      //change 
const int BUTTON_LEGS_DOWN = 1;    //change

class LegActuator : public rclcpp::Node
{
  public:
    LegActuator()
        : Node("leg_actuator"),
          // Initialize motors in the member initializer list
          motor_leg_bl_(SERIAL_NUMBER_1, PORT_0),
          motor_leg_br_(SERIAL_NUMBER_1, PORT_1),
          motor_leg_fl_(SERIAL_NUMBER_2, PORT_2),
          motor_leg_fr_(SERIAL_NUMBER_2, PORT_3)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Leg Actuator node...");
        
        // Initialize leg motors
        motor_leg_bl_.init();
        motor_leg_br_.init();
        motor_leg_fl_.init();
        motor_leg_fr_.init();
        
        RCLCPP_INFO(this->get_logger(), "All 4 leg motors initialized.");

        // Create the subscriber to /joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&LegActuator::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /joy.");
    }
   
    ~LegActuator()
    {
        // Ensure motors are stopped and cleaned up
        motor_leg_bl_.setVelocity(0.0);
        motor_leg_br_.setVelocity(0.0);
        motor_leg_fl_.setVelocity(0.0);
        motor_leg_fr_.setVelocity(0.0);
        
        motor_leg_bl_.cleanup();
        motor_leg_br_.cleanup();
        motor_leg_fl_.cleanup();
        motor_leg_fr_.cleanup();
        RCLCPP_INFO(this->get_logger(), "Motors stopped and cleaned up.");
    }

  private:
    // Callback for joy topic to control legs
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        double leg_velocity = 0.0;

        // Check if the "UP" button is pressed
        if (msg->buttons[BUTTON_LEGS_UP] == 1)
        {
            leg_velocity = LEG_MOVE_SPEED;
        }
        // Check if the "DOWN" button is pressed
        else if (msg->buttons[BUTTON_LEGS_DOWN] == 1)
        {
            leg_velocity = -LEG_MOVE_SPEED;
        }

        // Send command to all four leg motors
        // (Assuming they all move in the same direction)
        motor_leg_bl_.setVelocity(leg_velocity);
        motor_leg_br_.setVelocity(leg_velocity);
        motor_leg_fl_.setVelocity(leg_velocity);
        motor_leg_fr_.setVelocity(leg_velocity);
    }

    // Member variables for motors
    PhidgetActuationMotorController motor_leg_bl_; // motor0
    PhidgetActuationMotorController motor_leg_br_; // motor1
    PhidgetActuationMotorController motor_leg_fl_; // motor2
    PhidgetActuationMotorController motor_leg_fr_; // motor3

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegActuator>());
  rclcpp::shutdown();
  return 0;
}