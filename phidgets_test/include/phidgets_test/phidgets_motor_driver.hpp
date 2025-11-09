#ifndef PHIDGET_MOTOR_CONTROLLER_HPP
#define PHIDGET_MOTOR_CONTROLLER_HPP

#include <phidget22.h>
#include <cmath>

enum class MotorType {
    Drive,
    Actuation
};

struct MotorConfig {
    int max_rpm;
    double rescale_factor;
    double stall_velocity;
    double duty_limit;
};

class PhidgetMotorController {
public:
    PhidgetMotorController(int serial_number, int hub_port, MotorType type, int motor_direction)
        : serial_number_(serial_number), hub_port_(hub_port),  motor_direction_(motor_direction), motor_(nullptr) 
    {
        config_ = (type == MotorType::Drive) ? DRIVE_CONFIG : ACT_CONFIG;
    }

    ~PhidgetMotorController() {
        cleanup();
    }

    void init() {
        PhidgetBLDCMotor_create(&motor_);
        Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(motor_), serial_number_);
        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_), hub_port_);
        Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_), 5000);

        PhidgetBLDCMotor_setRescaleFactor(motor_, config_.rescale_factor);
        PhidgetBLDCMotor_setStallVelocity(motor_, config_.stall_velocity);
    }

    void setVelocityDuty(double duty) {
        duty = clampDuty(duty);
        PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
    }

    void setVelocityRPM(double rpm) {
        double duty = clampDuty(rpmToDuty(rpm));
        PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
    }

    void setVelocityRads(double rads_per_sec) {
        double duty = clampDuty(radsPerSecToDuty(rads_per_sec));
        PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
    }

    double getPositionDegs() {
        double pos;
        PhidgetBLDCMotor_getPosition(motor_, &pos);
        return pos;
    }

    double getPositionRads() {
        double pos;
        PhidgetBLDCMotor_getPosition(motor_, &pos);
        return degToRad(pos);
    }

    void cleanup() {
        if (!motor_) return;
        PhidgetBLDCMotor_setTargetVelocity(motor_, 0.0);
        Phidget_close(reinterpret_cast<PhidgetHandle>(motor_));
        PhidgetBLDCMotor_delete(&motor_);
        motor_ = nullptr;
    }

private:
    static constexpr double TWO_PI = 2.0 * M_PI;
    static constexpr double SECONDS_PER_MINUTE = 60.0;
    static constexpr double DEG_TO_RAD_FACTOR = M_PI / 180.0;

    // Conversion helper functions
    double degToRad(double degrees) const {
        return degrees * DEG_TO_RAD_FACTOR;
    }

    double rpmToDuty(double rpm) const {
        return rpm / config_.max_rpm;
    }

    double radsPerSecToDuty(double rads_per_sec) const {
        return (rads_per_sec * SECONDS_PER_MINUTE) / (TWO_PI * config_.max_rpm);
    }

    double clampDuty(double duty) const {
        if (duty > config_.duty_limit) return config_.duty_limit;
        if (duty < -config_.duty_limit) return -config_.duty_limit;
        return duty;
    }

    int serial_number_;
    int hub_port_;
    int motor_direction_;  
    PhidgetBLDCMotorHandle motor_;
    MotorConfig config_;

    static constexpr MotorConfig DRIVE_CONFIG {
        82,
        0.638297872340426,
        5.0,    // no idea if this works
        0.3,
    };

    static constexpr MotorConfig ACT_CONFIG {
        170,
        0.500, // update
        2.0,   // no idea if this works 
        0.15,  // update 
    };
};

#endif // PHIDGET_MOTOR_CONTROLLER_HPP