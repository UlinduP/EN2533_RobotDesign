#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PID.hpp>
#include <iostream>

#define TIME_STEP 16
#define MAX_SPEED 5.0

using namespace webots;

// Motor names
const int NUM_MOTORS = 4;
const char *motorNames[] = {"right_front_wheel", "right_back_wheel","left_front_wheel", "left_back_wheel"};


//IR sensor names
const int NUM_IR_SENSORS = 6;
const char *irSensorNames[] = {"ir_ext_right", "ir_right", "ir_middle_right", "ir_middle_left", "ir_left", "ir_ext_left"};

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // Initialize motors
  Motor *motors[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  // Initialize IR sensors
  DistanceSensor *irSensors[NUM_IR_SENSORS];
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
    irSensors[i]->enable(TIME_STEP);
  }

  double ir_error[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize IR errors to 0
  double ir_weight[6] = {-1.0, -0.5, -0.2, 0.2, 0.5, 1.0}; // define IR weights

  PID pid(TIME_STEP);
  pid.setGains(0.2, 0.0002, 3.0); // set the PID gains (Kp, Ki, Kd)
  pid.setRange(-MAX_SPEED, MAX_SPEED); // set the output range of the PID controller

  while (robot->step(TIME_STEP) != -1) {
    double ir_sum = 0.0;
    for (int i = 0; i < 6; i++) {
      ir_error[i] = (1 - ds[i]->getValue() / ds[i]->getMaxValue()) * ir_weight[i]; // calculate IR error
      ir_sum += ir_error[i];
    }
    double pid_output = pid.compute(ir_sum); // compute PID output
    leftMotor->setVelocity(MAX_SPEED + pid_output); // set left motor velocity
    rightMotor->setVelocity(MAX_SPEED - pid_output); // set right motor velocity
  }

  delete robot;
  return 0;
}
