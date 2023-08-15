#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

int main(int argc, char **argv) {
    // Create the robot object
    Robot *robot = new Robot();

    // Initialize motors
    Motor *left_front_wheel = robot->getMotor("left_front_wheel");
    Motor *left_back_wheel = robot->getMotor("left_back_wheel");
    Motor *right_front_wheel = robot->getMotor("right_front_wheel");
    Motor *right_back_wheel = robot->getMotor("right_back_wheel");
    
    left_front_wheel->setPosition(INFINITY);
    left_back_wheel->setPosition(INFINITY);
    right_front_wheel->setPosition(INFINITY);
    right_back_wheel->setPosition(INFINITY);
  
    left_front_wheel->setVelocity(0);
    left_back_wheel->setVelocity(0);
    right_front_wheel->setVelocity(0);
    right_back_wheel->setVelocity(0);

    // Initialize IR sensors
    DistanceSensor *ir_ext_right = robot->getDistanceSensor("ir_ext_right");
    DistanceSensor *ir_right = robot->getDistanceSensor("ir_right");
    DistanceSensor *ir_middle_right = robot->getDistanceSensor("ir_middle_right");
    DistanceSensor *ir_middle_left = robot->getDistanceSensor("ir_middle_left");
    DistanceSensor *ir_left = robot->getDistanceSensor("ir_left");
    DistanceSensor *ir_ext_left = robot->getDistanceSensor("ir_ext_left");

    // Enable IR sensors
    ir_ext_right->enable(10);
    ir_right->enable(10);
    ir_middle_right->enable(10);
    ir_middle_left->enable(10);
    ir_left->enable(10);
    ir_ext_left->enable(10);

    // Set wheel velocities
    double left_speed = 5.0;
    double right_speed = 5.0;
    
     // Set the target speed for the motors
    double target_speed = 2.0;

    // Main control loop
    while  (robot->step(16) != -1){
    
    
        // Read the sensor values
        double s1 = ir_ext_right->getValue();
        double s2 = ir_right->getValue();
        double s3 = ir_middle_right->getValue();
        double s4 = ir_middle_left->getValue();
        double s5 = ir_left->getValue();
        double s6 = ir_ext_left->getValue();

        // Calculate the error between the two sensor values
        double error = s1 * (-3.0) + s2 * (-1.5) + s3 * (-0.5) + s4 * 0.5 + s5 * 1.5 + s6 * 3.0;

        // Calculate the motor speeds based on the error
        double left_speed = target_speed - 0.01 * error;
        double right_speed = target_speed + 0.01 * error;

        // Set wheel velocities
        left_front_wheel->setVelocity(left_speed);
        left_back_wheel->setVelocity(left_speed);
        right_front_wheel->setVelocity(right_speed);
        right_back_wheel->setVelocity(right_speed);
    }

    // Cleanup
    delete robot;
    return 0;
}
