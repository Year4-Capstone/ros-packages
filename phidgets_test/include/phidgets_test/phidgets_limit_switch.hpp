#ifndef PHIDGET_LIMIT_SWITCH_HPP
#define PHIDGET_LIMIT_SWITCH_HPP

#include <phidget22.h>
#include <iostream>
#include <functional>
#include <stdexcept>
#include <string>
#include <atomic>

class PhidgetLimitSwitch {
public:
    PhidgetLimitSwitch(int serial_number, int hub_port)
        : serial_number_(serial_number),
          hub_port_(hub_port),
          digital_input_(nullptr),
          current_state_(false) {}

    ~PhidgetLimitSwitch() {
        cleanup();
    }

    // Initialize the digital input
    void init() {
        PhidgetReturnCode ret;

        ret = PhidgetDigitalInput_create(&digital_input_);
        checkError(ret, "Failed to create DigitalInput");

        ret = Phidget_setIsHubPortDevice(reinterpret_cast<PhidgetHandle>(digital_input_), 1);
        checkError(ret, "Failed to set hub port device");

        ret = Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(digital_input_), hub_port_);
        checkError(ret, "Failed to set hub port");

        ret = Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(digital_input_), serial_number_);
        checkError(ret, "Failed to set serial number");

        ret = PhidgetDigitalInput_setOnStateChangeHandler(PhidgetDigitalInputHandle(digital_input_), onStateChange, this);
        checkError(ret, "Failed to set state change handler");

        ret = Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(digital_input_), 5000);
        checkError(ret, "Failed to attach device");

        

    }

    // Manual cleanup
    void cleanup() {
        if (digital_input_) {
            Phidget_close(reinterpret_cast<PhidgetHandle>(digital_input_));
            PhidgetDigitalInput_delete(&digital_input_);
            digital_input_ = nullptr;
        }
    }

    bool read() const {
        return current_state_.load();
    }

    void setCallback(std::function<void(int)> cb) {
        callback_ = cb;
    }

private:

    static void CCONV onStateChange(PhidgetDigitalInputHandle ch, void* ctx, int state) {
        (void)ch;
        
        auto* self = static_cast<PhidgetLimitSwitch*>(ctx);
        if (!self) return;

        self->current_state_.store(state);

        if (self->callback_) {
            self->callback_(state);
        }
    }

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

    int serial_number_;
    int hub_port_;
    PhidgetDigitalInputHandle digital_input_;
    std::atomic<bool> current_state_;
    std::function<void(int)> callback_;
};

#endif // PHIDGET_LIMIT_SWITCH_HPP