#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

using namespace webots;

#define TIME_STEP 16
#define SPEED 3.0
#define MAX_SENSOR_DISTANCE 0.97

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    // Initialize left and right sonar sensors
    DistanceSensor *sonar_front_left = robot->getDistanceSensor("sonar_front_left");
    DistanceSensor *sonar_left = robot->getDistanceSensor("sonar_left");
    DistanceSensor *sonar_front_right = robot->getDistanceSensor("sonar_front_right");
    DistanceSensor *sonar_right = robot->getDistanceSensor("sonar_right");
    
    sonar_front_left->enable(TIME_STEP);
    sonar_left->enable(TIME_STEP);
    sonar_front_right->enable(TIME_STEP);
    sonar_right->enable(TIME_STEP);

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

    while (robot->step(TIME_STEP) != -1) {
        double distance_left = sonar_left->getValue()/1024;
        double distance_right = sonar_right->getValue()/1024;
        double distance_front_left = sonar_front_left->getValue()/1024;
        double distance_front_right = sonar_front_right->getValue()/1024;
        std::cout<<distance_right<<std::endl;

        if (distance_left > MAX_SENSOR_DISTANCE && distance_right > MAX_SENSOR_DISTANCE && distance_front_left > MAX_SENSOR_DISTANCE && distance_front_right > MAX_SENSOR_DISTANCE ) {
            // No wall in front, go straight
            left_front_wheel->setVelocity(SPEED);
            left_back_wheel->setVelocity(SPEED);
            right_front_wheel->setVelocity(SPEED);
            right_back_wheel->setVelocity(SPEED);
        } else if (distance_left > distance_right || distance_front_left > distance_front_right ) {
            // Turn left
            left_front_wheel->setVelocity(SPEED);
            left_back_wheel->setVelocity(SPEED);
            right_front_wheel->setVelocity(0.5*SPEED);
            right_back_wheel->setVelocity(0.5*SPEED);

        } else if (distance_left < distance_right || distance_front_left < distance_front_right ){
            // Turn right
            left_front_wheel->setVelocity(0.5*SPEED);
            left_back_wheel->setVelocity(0.5*SPEED);
            right_front_wheel->setVelocity(SPEED);
            right_back_wheel->setVelocity(SPEED);
        }
    }

    delete robot;
    return 0;
}
