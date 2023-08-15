#include <webots/Robot.hpp>
#include <webots/ColorSensor.hpp>
#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  ColorSensor *colorSensor = robot->getColorSensor("color_sensor");

  while (robot->step(64) != -1) {
    int red = colorSensor->getRed();
    int green = colorSensor->getGreen();
    int blue = colorSensor->getBlue();

    std::cout << "Red: " << red << ", Green: " << green << ", Blue: " << blue << std::endl;
  }

  delete robot;
  return 0;
}
