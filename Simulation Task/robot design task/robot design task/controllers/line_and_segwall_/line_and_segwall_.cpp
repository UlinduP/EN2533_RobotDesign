#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include <iostream>

using namespace webots;

static const int TIME_STEP = 16;
static const int MAX_SPEED = 10;

  double previousError = 0.0;
  double kp = 4.0;
  double kd = 1.0;
  double ki = 0.0;
  double Integral = 0.0;
  int readings[6] = {0};

double PID() {
  double error = 0;
  std::vector<int> coefficients = {-3000, -2000, -1000, 1000, 2000, 3000};
  for (int i = 0; i < irReadings.size(); ++i) {
    error += coefficients[i] * irReadings[i];
  }

  double P = kp * error;
  double I = Integral + (ki * error);
  double D = kd * (error - previousError);
  double correction = (P + I + D) / 1000.0;

  double l_speed = 5.0 + correction;
  double r_speed = 5.0 - correction;

  if (l_speed < 0.0) {l_speed = 0.0;}
  if (l_speed > 10.0) {l_speed = 10.0;}
  if (r_speed < 0.0) {r_speed = 0.0;}
  if (r_speed > 10.0) {r_speed = 10.0;}
  motors[0]->setVelocity(r_speed);
  motors[1]->setVelocity(r_speed);
  motors[2]->setVelocity(l_speed);
  motors[3]->setVelocity(l_speed);
  return I;
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  DistanceSensor *ir[6];
  const char *ir_names[6] = {"ir_ext_left", "ir_left", "ir_middle_left", "ir_middle_right", "ir_right", "ir_ext_right"};
  int ir_readings[6] = {0};
  for (int i = 0; i < 6; ++i) {
    ir[i] = robot->getDistanceSensor(ir_names[i]);
    ir[i]->enable(TIME_STEP);
  }
  
  DistanceSensor *dis[4];
  const char *dis_names[4] = {"dis_left", "dis_front_left", "dis_front_right", "dis_right"};
  int disReadings[6] = {0};
  for (int i = 0; i < 4; ++i) {
    dis[i] = robot->getDistanceSensor(dis_names[i]);
    dis[i]->enable(TIME_STEP);
  }

  Motor *wheel[4];
  const char *wheel_names[4] = {"right_front_wheel", "right_back_wheel", "left_front_wheel", "left_back_wheel"};
  for (int i = 0; i < 4; ++i) {
    wheel[i] = robot->getMotor(wheel_names[i]);
    wheel[i]->setPosition(INFINITY);
    wheel[i]->setVelocity(0.0);
  }



  while (robot->step(TIME_STEP) != -1) {
    for (int i = 0; i < 6; ++i) {
      if (ir[i]->getValue() > 512) {
        ir_readings[i] = 0;
      } else if (ir[i]->getValue() < 180) {
        ir_readings[i] = 1;
      }
    }

    if (ir_readings[1] == 1 || ir_readings[2] == 1 || ir_readings[3] == 1 || ir_readings[4] == 1) {
      for (int i = 0; i < 6; ++i) {
        readings[i] = ir_readings[i];
      }
    } else {
      if (dis[1]->getValue() > dis[2]->getValue() && dis[0]->getValue() > 400) {
        disReadings[1] = disReadings[2] = 1;
        disReadings[3] = disReadings[4] = 0;
      } else if (dis[1]->getValue() > 900 && dis[2]->getValue() > 900 && dis[0]->getValue() > 500 && dis[3]->getValue() > 500) {
        disReadings[1] = disReadings[4] = 0;
        disReadings[3] = disReadings[2] = 1;
      } else if (disReadings[1] > 900 && disReadings[2] > 900 && disReadings[0] > 500 && disReadings[3] > 500) {
        readings[1] = readings[2] = 1;
        readings[3] = readings[4] = 0;
        Integral = PID();
      } else if (disReadings[1] > disReadings[2] && disReadings[0] > 400) {
        readings[1] = readings[2] = 1;
        readings[3] = readings[4] = 0;
        Integral = PID();
      } else if (disReadings[2] > disReadings[1] || disReadings[0] < disReadings[3]) {
        readings[1] = readings[2] = 0;
        readings[3] = readings[4] = 1;
        Integral = PID();
      } else {
        Integral = 0.0;
      }
    }
    
    delete robot;
    return 0;
}
    