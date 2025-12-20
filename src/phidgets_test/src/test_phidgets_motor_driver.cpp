#include "rclcpp/rclcpp.hpp"
#include "phidgets_test/phidgets_motor_driver.hpp"
#include <signal.h>

const int SERIAL_NUMBER_1 = 527103;
const int SERIAL_NUMBER_2 = 527164;
const int PORT_0 = 0;
const int PORT_1 = 1; 
const int PORT_2 = 2;
const int PORT_3 = 3; 
const double VELOCITY = 0.1;
const int NUM_MOTORS = 4;

int main()
{
    PhidgetMotorController motor_drive_bl(SERIAL_NUMBER_1, PORT_2, MotorType::Drive, +1); 
    PhidgetMotorController motor_drive_br(SERIAL_NUMBER_1, PORT_3, MotorType::Drive, -1);
    PhidgetMotorController motor_drive_fl(SERIAL_NUMBER_2, PORT_2, MotorType::Drive, +1);
    PhidgetMotorController motor_drive_fr(SERIAL_NUMBER_2, PORT_3, MotorType::Drive, -1);

    PhidgetMotorController motor_actuation_bl(SERIAL_NUMBER_1, PORT_0, MotorType::Actuation, +1); 
    PhidgetMotorController motor_actuation_br(SERIAL_NUMBER_1, PORT_1, MotorType::Actuation, +1);
    PhidgetMotorController motor_actuation_fl(SERIAL_NUMBER_2, PORT_0, MotorType::Actuation, +1);
    PhidgetMotorController motor_actuation_fr(SERIAL_NUMBER_2, PORT_1, MotorType::Actuation, +1);
    
    PhidgetMotorController* motors_drive[] = {&motor_drive_bl, &motor_drive_br, &motor_drive_fl, &motor_drive_fr};
    PhidgetMotorController* motors_actuation[] = {&motor_actuation_bl, &motor_actuation_br, &motor_actuation_fl, &motor_actuation_fr};

    // Initialize all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors_drive[i]->init();
        motors_actuation[i]->init();
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        double position = motors_drive[i]->getPositionRads();
        printf("Motor %d Position: %f\n", i, position);
        printf(" ");
    }
    
    // Set velocity of drive motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors_drive[i]->setVelocityDuty(0.2);
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        double position = motors_drive[i]->getPositionRads();
        printf("Motor %d Position: %f\n", i, position);
        printf(" ");
    }
    
    sleep(5);

    for (int i = 0; i < NUM_MOTORS; i++) {
        double position = motors_drive[i]->getPositionRads();
        printf("Motor %d Position: %f\n", i, position);
        printf(" ");
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors_drive[i]->cleanup();
        motors_actuation[i]->cleanup();
    }
    
    return 0;
}