#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;

int main(int argc, char **argv) {
    // Create the robot object
    Robot *robot = new Robot();

    // Initialize motors
    Motor *left_front_wheel = robot->getMotor("left_front_wheel");
    Motor *left_back_wheel = robot->getMotor("left_back_wheel");
    Motor *right_front_wheel = robot->getMotor("right_front_wheel");
    Motor *right_back_wheel = robot->getMotor("right_back_wheel");
    
    // Enabling the morotors
    left_front_wheel->setPosition(INFINITY);
    left_back_wheel->setPosition(INFINITY);
    right_front_wheel->setPosition(INFINITY);
    right_back_wheel->setPosition(INFINITY);


    // Main control loop
    while  (robot->step(16) != -1){
        // Set wheel velocities
        left_front_wheel->setVelocity(5.0);
        left_back_wheel->setVelocity(5.0);
        right_front_wheel->setVelocity(5.0);
        right_back_wheel->setVelocity(5.0);
    }

    // Cleanup
    delete robot;
    return 0;
}
