#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

using namespace webots;

#define NUM_IR 6

int main(int argc, char **argv) {
    // Create the robot object
    Robot *robot = new Robot();

    // Initialize IR sensors
    DistanceSensor *irSensors[NUM_IR];
    char irSensorNames[NUM_IR][32] = {"ir_ext_right", "ir_right", "ir_middle_right", "ir_middle_left", "ir_left", "ir_ext_left"};

    for (int i = 0; i < NUM_IR; i++) {
        irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
        irSensors[i]->enable(10); // Enable the sensor with a sampling period of 10ms
    }

    // Main control loop
    while (robot->step(10) != -1) {
        // Read and print sensor values
        for (int i = 0; i < NUM_IR; i++) {
            std::cout << irSensorNames[i] << ": " << irSensors[i]->getValue() << std::endl;
        }
    }

    // Cleanup
    delete robot;
    return 0;
}
