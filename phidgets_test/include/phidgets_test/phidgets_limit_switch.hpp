#ifndef PHIDGET_LIMIT_SWITCH_HPP
#define PHIDGET_LIMIT_SWITCH_HPP

#include <phidget22.h>

class PhidgetLimitSwitch {
public:
    PhidgetLimitSwitch(int serial_number, int hub_port)
        : serial_number_(serial_number),
          hub_port_(hub_port),
          digital_input_(nullptr) {}

    ~PhidgetLimitSwitch() {
        cleanup();
    }

    // Initialize the digital input
    void init() {
        PhidgetDigitalInput_create(&digital_input_);
        Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(digital_input_), serial_number_);
        Phidget_setHubPort(reinterpret_cast<PhidgetHandle>(digital_input_), hub_port_);
        Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(digital_input_), 5000);
    }

    // Read the state of the limit switch
    bool read() {
        int state;
        PhidgetDigitalInput_getState(digital_input_, &state);
        return static_cast<bool>(state);
    }

    // Manual cleanup
    void cleanup() {
        if (digital_input_) {
            Phidget_close(reinterpret_cast<PhidgetHandle>(digital_input_));
            PhidgetDigitalInput_delete(&digital_input_);
            digital_input_ = nullptr;
        }
    }

private:
    int serial_number_;
    int hub_port_;
    PhidgetDigitalInputHandle digital_input_;
};

#endif // PHIDGET_LIMIT_SWITCH_HPP