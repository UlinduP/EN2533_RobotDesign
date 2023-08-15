#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  // create a Robot instance
  Robot *robot = new Robot();

  // enable all distance sensors
  DistanceSensor *sensors[4];
  sensors[0] = robot->getDistanceSensor("sonar_right");
  sensors[1] = robot->getDistanceSensor("sonar_front_right");
  sensors[2] = robot->getDistanceSensor("sonar_front_left");
  sensors[3] = robot->getDistanceSensor("sonar_left");
  for (int i = 0; i < 4; i++)
    sensors[i]->enable(100);

  while (robot->step(64) != -1) {
    double sonarValues[4];
    for (int i = 0; i < 4; i++)
      sonarValues[i] = sensors[i]->getValue();
    std::cout << "Sonar values: " << sonarValues[0] << " " << sonarValues[1] << " " << sonarValues[2] << " " << sonarValues[3] << std::endl;
  }

  delete robot;
  return 0;
}
