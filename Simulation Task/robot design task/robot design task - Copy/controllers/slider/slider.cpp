#include <string>
#include <vector>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>
#define MAX_SPEED 7
#define MAX_SPEED_line_follow 7
#define NOM_SPEED_MAZE 4
#define MAX_SPEED_MAZE 9
#define TIME_STEP 16

using namespace webots;
using namespace std;




int main(int argc, char **argv){
    Robot *robot = new Robot();
    Motor *linear_motor = robot->getMotor("linear_motor");
    linear_motor->setPosition(INFINITY);
    linear_motor->setVelocity(0.0);
    PositionSensor *linear_sensor = robot->getPositionSensor("linear_sensor");
    linear_sensor->enable(TIME_STEP);
    
    double i=0;
    while (robot->step(TIME_STEP) != -1) {
        linear_motor->setPosition(i);
        i+=0.1;
    }



    delete robot;
    return 0;

}