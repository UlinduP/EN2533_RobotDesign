#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/ColorSensor.hpp>
#include <iostream>

using namespace webots;

int main(int argc, char **argv) {
  Robot robot;
  
  // Initialize color sensor
  ColorSensor colorSensor("colorSensor");
  colorSensor.enable(10); // Enable sensor with 10ms update interval
  
  // Define reference RGB values for target color
  int targetR = 255;
  int targetG = 0;
  int targetB = 0;
  
  // Define tolerance for color matching
  int tolerance = 10;
  
  while (robot.step(10) != -1) {
    // Read RGB values from color sensor
    const unsigned char *rgb = colorSensor.getImage();
    int r = rgb[0];
    int g = rgb[1];
    int b = rgb[2];
    
    // Compare RGB values to reference values
    if (r > targetR - tolerance && r < targetR + tolerance
        && g > targetG - tolerance && g < targetG + tolerance
        && b > targetB - tolerance && b < targetB + tolerance) {
      std::cout << "Target color detected!" << std::endl;
    } else {
      std::cout << "Target color not detected." << std::endl;
    }
    
    // Release memory allocated for RGB data
    colorSensor.imageDelete(rgb);
  }
  
  return 0;
}
