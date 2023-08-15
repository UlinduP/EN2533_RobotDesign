#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define NUM_IR 6

using namespace webots;

int main(int argc, char **argv) {
    // Create the robot object
    Robot *robot = new Robot();

    // Initialize motors and sensors
    Motor *leftMotor1 = robot->getMotor("left_motor_1");
    Motor *leftMotor2 = robot->getMotor("left_motor_2");
    Motor *rightMotor1 = robot->getMotor("right_motor_1");
    Motor *rightMotor2 = robot->getMotor("right_motor_2");
    DistanceSensor *irSensors[NUM_IR];
    char irSensorNames[NUM_IR][32] = {"ir_ext_right", "ir_right", "ir_middle_right", "ir_middle_left", "ir_left", "ir_ext_left"};
    for (int i = 0; i < NUM_IR; i++) {
        irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
        irSensors[i]->enable(10);
    }
    std::cout<<6<<std::endl;
    // Main control loop
    while (robot->step(10) != -1) {
        //std::cout<<6<<std::endl;
        // Read sensor values
        //double irValues[NUM_IR];
        //for (int i = 0; i < NUM_IR; i++) {
          //  irValues[i] = irSensors[i]->getValue();
        //}

        // Calculate error
        double error = 0.0;
      //  for (int i = 0; i < NUM_IR; i++) {
        //    error += (i - (NUM_IR - 1) / 2) * irValues[i];
        //}

        // Set wheel velocities
        double leftSpeed = 5.0 - 0.001 * error;
        double rightSpeed = 5.0 + 0.001 * error;
        leftMotor1->setVelocity(leftSpeed);
        leftMotor2->setVelocity(leftSpeed);
        rightMotor1->setVelocity(rightSpeed);
        rightMotor2->setVelocity(rightSpeed);
        std::cout<<leftSpeed;
        std::cout<<rightSpeed<<std::endl;
    }

    // Cleanup
    delete robot;
    return 0;
}
