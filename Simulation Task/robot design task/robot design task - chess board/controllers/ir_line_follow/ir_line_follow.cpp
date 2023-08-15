#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 16
#define NUM_IR 6
#define MAX_SPEED 6.28

using namespace webots;

double irSensorValues[NUM_IR];
double kp = 0.5;
double ki = 0.1;
double kd = 0.01;
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;
double desiredPosition = 0.5;

double pidControl() {
  error = desiredPosition - irSensorValues[0];
  integral += error;
  derivative = error - lastError;
  lastError = error;
  double output = kp * error + ki * integral + kd * derivative;
  return output;
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get motors devices
  Motor *left_front_motor = robot->getMotor("left_front_wheel");
  Motor *left_back_motor = robot->getMotor("left_back_wheel");
  Motor *right_front_motor = robot->getMotor("right_front_wheel");
  Motor *right_back_motor = robot->getMotor("right_back_wheel");
  
  //Initialize motors
  left_front_motor->setPosition(INFINITY);
  left_back_motor->setPosition(INFINITY);
  right_front_motor->setPosition(INFINITY);
  right_back_motor->setPosition(INFINITY);
  
  left_front_motor->setVelocity(0);
  left_back_motor->setVelocity(0);
  right_front_motor->setVelocity(0);
  right_back_motor->setVelocity(0);

 
  // Initialize IR sensors
  DistanceSensor *irSensors[NUM_IR];
  char irSensorNames[NUM_IR][32] = {"ir_ext_right", "ir_right", "ir_middle_right", "ir_middle_left", "ir_left", "ir_ext_left"};

  for (int i = 0; i < NUM_IR; i++) {
      irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
      irSensors[i]->enable(10); // Enable the sensor with a sampling period of 10ms
  }
    

  // main loop
  while (robot->step(TIME_STEP) != -1) {
    // read the IR sensor values
    for (int i = 0; i < NUM_IR; i++) {
      irSensorValues[i] = irSensors[i]->getValue() / 4096.0;
    }

    // calculate the PID output
    double pidOutput = pidControl();

    // adjust the motor speeds based on the PID output
    double leftSpeed = MAX_SPEED - pidOutput; 
    double rightSpeed = MAX_SPEED + pidOutput;
    left_front_motor->setVelocity(leftSpeed);
    left_back_motor->setVelocity(leftSpeed);
    right_front_motor->setVelocity(rightSpeed);
    right_back_motor->setVelocity(rightSpeed);
  }

  // cleanup code
  delete robot;
  return 0;
}
