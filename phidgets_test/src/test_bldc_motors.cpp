#include "rclcpp/rclcpp.hpp"
#include "phidgets_test/phidgets_bldc_driver.hpp"
#include <signal.h>

const int SERIAL_NUMBER_1 = 527103;
const int SERIAL_NUMBER_2 = 527164;
const int PORT_0 = 2; // back left
const int PORT_1 = 3; // back right
const int PORT_2 = 2; // front left
const int PORT_3 = 3; // front right
const double VELOCITY = 0.1;

int main()
{
    // Create 4 motor controllers
    PhidgetDriveMotorController motor0(SERIAL_NUMBER_1, PORT_0);
    PhidgetDriveMotorController motor1(SERIAL_NUMBER_1, PORT_1);
    PhidgetDriveMotorController motor2(SERIAL_NUMBER_2, PORT_2);
    PhidgetDriveMotorController motor3(SERIAL_NUMBER_2, PORT_3);
    
    // Initialize all motors
    motor0.init();
    motor1.init();
    motor2.init();
    motor3.init();
    
    // Or use an array for easier management
    PhidgetDriveMotorController* motors[] = {&motor0, &motor1, &motor2, &motor3};
    
    motor0.setVelocityDuty(0.1);
    motor1.setVelocityDuty(-0.1);
    motor2.setVelocityDuty(0.1);
    motor3.setVelocityDuty(-0.1);

    sleep(5);

    for (int i = 0; i < 4; i++) {
        motors[i]->cleanup();
    }
    
    return 0;
}