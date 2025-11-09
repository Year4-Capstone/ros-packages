#include "rclcpp/rclcpp.hpp"
#include "phidgets_test/phidgets_limit_switch.hpp"
#include <signal.h>
#include <unistd.h>

const int SERIAL_NUMBER_1 = 527103;
const int SERIAL_NUMBER_2 = 527164;
const int PORT_4 = 4;
const int PORT_5 = 5;

volatile sig_atomic_t shutdown_requested = 0;

void signalHandler(int signum) {
    (void)signum;
    shutdown_requested = 1;
}

int main()
{
    // Set up signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Create limit switch controllers - 2 on each hub
    PhidgetLimitSwitch limit_switch_hub1_port4(SERIAL_NUMBER_1, PORT_4);
    PhidgetLimitSwitch limit_switch_hub1_port5(SERIAL_NUMBER_1, PORT_5);
    PhidgetLimitSwitch limit_switch_hub2_port4(SERIAL_NUMBER_2, PORT_4);
    PhidgetLimitSwitch limit_switch_hub2_port5(SERIAL_NUMBER_2, PORT_5);
    
    // Initialize all limit switches
    printf("Initializing limit switches...\n");
    limit_switch_hub1_port4.init();
    limit_switch_hub1_port5.init();
    limit_switch_hub2_port4.init();
    limit_switch_hub2_port5.init();
    printf("All limit switches initialized!\n");
    
    printf("Reading limit switches. Press Ctrl+C to stop...\n\n");
    
    // Continuously read and display limit switch states
    while (!shutdown_requested) {
        bool hub1_port4 = limit_switch_hub1_port4.read();
        bool hub1_port5 = limit_switch_hub1_port5.read();
        bool hub2_port4 = limit_switch_hub2_port4.read();
        bool hub2_port5 = limit_switch_hub2_port5.read();
        
        printf("\rHub1 Port4: %s | Hub1 Port5: %s | Hub2 Port4: %s | Hub2 Port5: %s   ", 
               hub1_port4 ? "PRESSED" : "RELEASED",
               hub1_port5 ? "PRESSED" : "RELEASED",
               hub2_port4 ? "PRESSED" : "RELEASED",
               hub2_port5 ? "PRESSED" : "RELEASED");
        fflush(stdout);
        
        usleep(100000); // Sleep for 100ms
    }
    
    printf("\n\nShutting down limit switches...\n");
    
    // Manually cleanup all switches
    limit_switch_hub1_port4.cleanup();
    limit_switch_hub1_port5.cleanup();
    limit_switch_hub2_port4.cleanup();
    limit_switch_hub2_port5.cleanup();
    
    printf("Done!\n");
    
    return 0;
}