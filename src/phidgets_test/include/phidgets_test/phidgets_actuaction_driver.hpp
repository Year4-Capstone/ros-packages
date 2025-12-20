#ifndef PHIDGET_MOTOR_CONTROLLER_HPP
#define PHIDGET_MOTOR_CONTROLLER_HPP

#include <phidget22.h>

class PhidgetActuationMotorController {
public:
    PhidgetActuationMotorController(int serial_number, int hub_port)
        : serial_number_(serial_number),
          hub_port_(hub_port),
          motor_(nullptr) {}

    ~PhidgetActuationMotorController() {
        cleanup();
    }

    // Initialize and configure the motor
    void init() {
        PhidgetBLDCMotor_create(&motor_);
        Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(motor_), serial_number_);
        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_), hub_port_);
        Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_), 5000);
        PhidgetBLDCMotor_setRescaleFactor(motor_, 0.638297872340426); //update
        PhidgetBLDCMotor_setStallVelocity(motor_, 5.0); //update
    }

    // Set motor velocity
    void setVelocity(double velocity) {
        if (velocity > 0.3) velocity = 0.3;
        if (velocity < -0.3) velocity = -0.3;
        PhidgetBLDCMotor_setTargetVelocity(motor_, velocity);
    }

    void cleanup() {
        if (motor_) {
            PhidgetBLDCMotor_setTargetVelocity(motor_, 0.0);
            Phidget_close(reinterpret_cast<PhidgetHandle>(motor_));
            PhidgetBLDCMotor_delete(&motor_);
            motor_ = nullptr;
        }
    }

private:
    int serial_number_;
    int hub_port_;
    PhidgetBLDCMotorHandle motor_;


};

#endif // PHIDGET_MOTOR_CONTROLLER_HPP