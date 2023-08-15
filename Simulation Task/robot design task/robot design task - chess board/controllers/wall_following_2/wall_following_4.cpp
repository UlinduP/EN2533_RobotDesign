#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

using namespace webots;

#define TIME_STEP 16
#define SPEED 3.0
#define MAX_SENSOR_DISTANCE 0.5

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    // Initialize left and right sonar sensors
    DistanceSensor *leftSonar = robot->getDistanceSensor("leftSonar");
    leftSonar->enable(TIME_STEP);
    DistanceSensor *rightSonar = robot->getDistanceSensor("rightSonar");
    rightSonar->enable(TIME_STEP);

    // Initialize left and right motors
    Motor *leftMotor = robot->getMotor("leftMotor");
    Motor *rightMotor = robot->getMotor("rightMotor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(SPEED);
    rightMotor->setVelocity(SPEED);

    while (robot->step(TIME_STEP) != -1) {
        double leftDistance = leftSonar->getValue();
        double rightDistance = rightSonar->getValue();

        if (leftDistance > MAX_SENSOR_DISTANCE && rightDistance > MAX_SENSOR_DISTANCE) {
            // No wall in front, go straight
            leftMotor->setVelocity(SPEED);
            rightMotor->setVelocity(SPEED);
        } else if (leftDistance > rightDistance) {
            // Turn left
            leftMotor->setVelocity(SPEED);
            rightMotor->setVelocity(0.5*SPEED);
        } else if (leftDistance < rightDistance) {
            // Turn right
            leftMotor->setVelocity(0.5*SPEED);
            rightMotor->setVelocity(SPEED);
        }
    }

    delete robot;
    return 0;
}
