#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <vector>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  vector<DistanceSensor*> dis;
  vector<string> dis_Names = {"sonar_left", "sonar_front_left" , "sonar_front_right", "sonar_right"};
  vector<int> dis_readings(dis_Names.size()+2, 0);
  for (string name : dis_Names) {
    dis.push_back(robot->getDistanceSensor(name));
    dis.back()->enable(timeStep);
  }

  vector<Motor*> wheels;
  vector<string> wheel_Names = {"right_front_wheel", "right_back_wheel", "left_front_wheel", "left_back_wheel"};
  for (string name : wheel_Names) {
    wheels.push_back(robot->getMotor(name));
    wheels.back()->setPosition(INFINITY);
    wheels.back()->setVelocity(0.0);
  }

  double previous_error = 0.0;
  double kp = 6.0;
  double kd = 0.5;
  double ki = 0.0;
  double Integral = 0.0;

  while (robot->step(timeStep) != -1) {
    // get distance sensor readings
    for (int i = 0; i < 4; i++) {
      dis_readings[i] = (int)dis[i]->getValue();
    }

    // calculate wall following correction
    double error = 0.0;
    vector<int> coefficient = {-3000, -2000, -1000, 1000, 2000, 3000};
    for (int i = 0; i < 6; i++) {
      error += coefficient[i] * dis_readings[i];
    }
    double P = kp * error;
    Integral += ki * error;
    double D = kd * (error - previous_error);
    previous_error = error;
    double correction = (P + Integral + D) / 1000.0;

    // set wheel velocities
    double l_speed = 5 + correction;
    double r_speed = 5 - correction;
    if (l_speed < 0.0) l_speed = 0.0;
    if (l_speed > 10.0) l_speed = 10.0;
    if (r_speed < 0.0) r_speed = 0.0;
    if (r_speed > 10.0) r_speed = 10.0;
    wheels[0]->setVelocity(r_speed);
    wheels[1]->setVelocity(r_speed);
    wheels[2]->setVelocity(l_speed);
    wheels[3]->setVelocity(l_speed);

    // print readings, PID values, and wheel velocities
    for (int i = 0; i < 6; i++) {
      cout << dis_readings[i] << " ";
    }
    cout << endl;
    cout << "kp: " << kp << " kd: " << kd << " ki: " << ki << endl;
    cout << "l_speed: " << l_speed << " r_speed: " << r_speed << endl;
  }

  delete robot;
  return 0;
}
    
      
  