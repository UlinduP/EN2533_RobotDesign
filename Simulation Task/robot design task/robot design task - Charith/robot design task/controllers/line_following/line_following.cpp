#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  // Get handles to the line sensors and motors
  DistanceSensor *left_sensor = robot->getDistanceSensor("left_sensor");
  DistanceSensor *right_sensor = robot->getDistanceSensor("right_sensor");
  Motor *left_motor = robot->getMotor("left_motor");
  Motor *right_motor = robot->getMotor("right_motor");

  // Set the sensors to enable distance measurement
  left_sensor->enable(10);
  right_sensor->enable(10);

  // Set the target speed for the motors
  double target_speed = 2.5;

  // Start the main control loop
  while (robot->step(10) != -1) {

    // Read the sensor values
    double left_value = left_sensor->getValue();
    double right_value = right_sensor->getValue();

    // Calculate the error between the two sensor values
    double error = right_value - left_value;

    // Calculate the motor speeds based on the error
    double left_speed = target_speed - 0.05 * error;
    double right_speed = target_speed + 0.05 * error;

    // Set the motor speeds
    left_motor->setVelocity(left_speed);
    right_motor->setVelocity(right_speed);

  }

  delete robot;
  return 0;
}
