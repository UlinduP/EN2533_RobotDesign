#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 16
#define MAX_SPEED 2.0

using namespace webots;

int main(int argc, char **argv) {
  // Create the robot object
  Robot *robot = new Robot();

  // Initialize sonar sensors
  DistanceSensor *sonarSensors[4];
  sonarSensors[0] = robot->getDistanceSensor("sonar_left");
  sonarSensors[1] = robot->getDistanceSensor("sonar_front_left");
  sonarSensors[2] = robot->getDistanceSensor("sonar_front_right");
  sonarSensors[3] = robot->getDistanceSensor("sonar_right");
  for (int i = 0; i < 4; i++) {
    sonarSensors[i]->enable(TIME_STEP);
  }

  // Initialize motors
  Motor *motors[4];
  motors[0] = robot->getMotor("left_front_wheel");
  motors[1] = robot->getMotor("left_back_wheel");
  motors[2] = robot->getMotor("right_front_wheel");
  motors[3] = robot->getMotor("right_back_wheel");
  for (int i = 0; i < 4; i++) {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  // Main control loop
  while (robot->step(TIME_STEP) != -1) {
    // Read sonar sensor values
    double sonarSensorValues[4];
    for (int i = 0; i < 4; i++) {
      sonarSensorValues[i] = sonarSensors[i]->getValue();
    }

    // Compute the error as the sum of the differences between the front and back sensors
    double error = (sonarSensorValues[1] - sonarSensorValues[0]) +
                   (sonarSensorValues[3] - sonarSensorValues[2]);

    // Determine the correction required based on the error
    double correction = 0.0;
    if (error > 0.0) {
      correction = -0.5;
    } else if (error < 0.0) {
      correction = 0.5;
    }

    // Apply the correction to the motors
    motors[0]->setVelocity(MAX_SPEED + correction);
    motors[1]->setVelocity(MAX_SPEED + correction);
    motors[2]->setVelocity(MAX_SPEED - correction);
    motors[3]->setVelocity(MAX_SPEED - correction);
  }

  delete robot;
  return 0;
}
