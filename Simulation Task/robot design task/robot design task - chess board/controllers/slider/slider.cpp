#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#define MAX_SPEED 7
#define TIME_STEP 16

using namespace webots;
using namespace std;
double linear=0;
int main(int argc, char **argv){
    Robot *robot = new Robot();
    Motor *rotational_motor = robot->getMotor("rotational_motor");
    
  
  while (robot->step(TIME_STEP) != -1) 
   {
     cout<<"Laser sensor:"<<linear<<endl;
     linear+=0.05;
      
     if (linear>=5.0)
     {
       rotational_motor->setVelocity(0.0);
       break;
     }  
     rotational_motor->setPosition(linear);
      
    }
    delete robot;
    return 0;
}
