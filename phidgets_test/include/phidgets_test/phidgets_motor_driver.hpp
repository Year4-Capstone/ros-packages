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
        PhidgetReturnCode ret;
        ret = PhidgetBLDCMotor_create(&motor_);
        checkError(ret, "Failed to create BLDC motor");
        ret = Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(motor_), serial_number_);
        checkError(ret, "Failed to set device serial number");
        ret = Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(motor_), hub_port_);
        checkError(ret, "Failed to set hub port");
        ret = Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_), 5000);
        checkError(ret, "Failed to attach to motor");

        ret = PhidgetBLDCMotor_setStallVelocity(motor_, config_.stall_velocity);
        checkError(ret, "Failed to set rescale factor");
        ret = PhidgetBLDCMotor_setRescaleFactor(motor_, config_.rescale_factor);
        checkError(ret, "Failed to set stall velocity");
    }

    void setVelocityDuty(double duty) {
        duty = clampDuty(duty);
        PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
    }

    void setVelocityRPM(double rpm) {
        double duty = clampDuty(rpmToDuty(rpm));
        PhidgetReturnCode ret = PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
        checkError(ret, "Failed to set target velocity (RPM)");
    }

    void setVelocityRads(double rads_per_sec) {
        double duty = clampDuty(radsPerSecToDuty(rads_per_sec));
        PhidgetReturnCode ret = PhidgetBLDCMotor_setTargetVelocity(motor_, duty * motor_direction_);
        checkError(ret, "Failed to set target velocity (rad/s)");
    }

    double getPositionDegs() {
        double pos;
        PhidgetReturnCode ret = PhidgetBLDCMotor_getPosition(motor_, &pos);
        checkError(ret, "Failed to get position");
        return pos;
    }

    double getPositionRads() {
        double pos;
        PhidgetReturnCode ret = PhidgetBLDCMotor_getPosition(motor_, &pos);
        checkError(ret, "Failed to get position");
        return degToRad(pos);
    }

    void resetPosition() {
        double current_pos;
        PhidgetReturnCode ret = PhidgetBLDCMotor_getPosition(motor_, &current_pos);
        checkError(ret, "Failed to get position for reset");
        ret = PhidgetBLDCMotor_addPositionOffset(motor_, -current_pos);
        checkError(ret, "Failed to add position offset to reset position");
    }

    void cleanup() {
        if (!motor_) return;
            // Don't throw from cleanup since it's called in destructor
            // Just log errors or store them
            PhidgetReturnCode ret;
            
            ret = PhidgetBLDCMotor_setTargetVelocity(motor_, 0.0);
            if (ret != EPHIDGET_OK) {
                // Log error but continue cleanup
                fprintf(stderr, "Warning: Failed to stop motor during cleanup\n");
            }
            
            ret = Phidget_close(reinterpret_cast<PhidgetHandle>(motor_));
            if (ret != EPHIDGET_OK) {
                fprintf(stderr, "Warning: Failed to close motor during cleanup\n");
            }
            
            ret = PhidgetBLDCMotor_delete(&motor_);
            if (ret != EPHIDGET_OK) {
                fprintf(stderr, "Warning: Failed to delete motor handle during cleanup\n");
            }
            
            motor_ = nullptr;
    }

private:

    static constexpr double TWO_PI = 2.0 * M_PI;
    static constexpr double SECONDS_PER_MINUTE = 60.0;
    static constexpr double DEG_TO_RAD_FACTOR = M_PI / 180.0;

    void checkError(PhidgetReturnCode ret, const std::string& context) {
        if (ret != EPHIDGET_OK) {
            PhidgetReturnCode errorCode;
            const char* errorString;
            char errorDetail[100];
            size_t errorDetailLen = 100;
            
            Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
            
            std::string error_msg = context + " - Error (" + std::to_string(errorCode) + 
                                   "): " + errorString;
            if (errorDetailLen > 0) {
                error_msg += " - " + std::string(errorDetail);
            }
            
            throw std::runtime_error(error_msg);
        }
    }

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