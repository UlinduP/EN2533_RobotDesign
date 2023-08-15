#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 16
#define MAX_SPEED 2.0

using namespace webots;

// Sonar sensor names
const char *sonarSensorNames[] = {
  "sonar_left", "sonar_front_left", "sonar_front_right", "sonar_right"
};
const int NUM_SONAR_SENSORS = 4;

// Motor names
const char *motorNames[] = {
  "left_front_wheel", "left_back_wheel",
  "right_front_wheel", "right_back_wheel"
};
const int NUM_MOTORS = 4;

int main(int argc, char **argv) {
  // Create the robot object
  Robot *robot = new Robot();

  // Initialize sonar sensors
  DistanceSensor *sonarSensors[NUM_SONAR_SENSORS];
  for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
    sonarSensors[i] = robot->getDistanceSensor(sonarSensorNames[i]);
    sonarSensors[i]->enable(TIME_STEP);
  }

  // Initialize motors
  Motor *motors[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  // Main control loop
  while (robot->step(TIME_STEP) != -1) {
    // Read sonar sensor values
    double sonarSensorValues[NUM_SONAR_SENSORS];
    for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
      sonarSensorValues[i] = sonarSensors[i]->getValue();
      std::cout<<i;
      std::cout<<'-';
      std::cout<<sonarSensorValues[i]<<std::endl;
    }

    // Calculate motor speeds based on sonar sensor values
    double leftSpeed = MAX_SPEED;
    double rightSpeed = MAX_SPEED;
    double frontDistance = sonarSensorValues[1] + sonarSensorValues[2];
    if (frontDistance < 200) {
      leftSpeed = -MAX_SPEED;
      rightSpeed = -MAX_SPEED;
    } else if (sonarSensorValues[0] < 300) {
      leftSpeed = MAX_SPEED/8;
      rightSpeed = MAX_SPEED;
    } else if (sonarSensorValues[3] < 300) {
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED/8;
    } else if (sonarSensorValues[0] < sonarSensorValues[3]) {
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED/4;
    } else if (sonarSensorValues[0] > sonarSensorValues[3]) {
      leftSpeed = MAX_SPEED / 4;
      rightSpeed = MAX_SPEED;
    }

    // Set motor speeds
    motors[0]->setVelocity(leftSpeed);
    motors[1]->setVelocity(leftSpeed);
    motors[2]->setVelocity(rightSpeed);
    motors[3]->setVelocity(rightSpeed);
  }

  delete robot;
  return 0;
}
