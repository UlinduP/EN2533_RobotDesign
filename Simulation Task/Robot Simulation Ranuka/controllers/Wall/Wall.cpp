#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 32

using namespace webots;
  int value;
  int speed=7;
  double error,previous_error=0,integral,derivative,PID_error;
  double Kp=0.5;
  double Ki=0.01;
  double Kd=0.01;
  double max_speed=30.0;
  double min_speed=30.0;
  int leftmotor_speed,rightmotor_speed;
  double error1,error2;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  
  
  // Get the front proximity sensor
  DistanceSensor *frontSensor = robot->getDistanceSensor("Front");
  frontSensor->enable(TIME_STEP);
  
  
  // Get the side proximity sensors
  DistanceSensor *leftSensor1 = robot->getDistanceSensor("Left_1");
  leftSensor1->enable(TIME_STEP);
  DistanceSensor *rightSensor1 = robot->getDistanceSensor("Right_1");
  rightSensor1->enable(TIME_STEP);
  DistanceSensor *leftSensor2=robot->getDistanceSensor("Left_2");
  leftSensor2-> enable(TIME_STEP);
  DistanceSensor *rightSensor2 =robot->getDistanceSensor("Right_2");
  rightSensor2-> enable(TIME_STEP);
  
  // Get the wheel motors
  Motor *leftWheel1 = robot->getMotor("leftTopMotor");
  Motor *leftWheel2 = robot->getMotor("leftRearMotor");
  Motor *rightWheel1 = robot->getMotor("rightTopMotor");
  Motor *rightWheel2 = robot->getMotor("rightRearMotor");
  leftWheel1->setPosition(INFINITY);
  leftWheel2->setPosition(INFINITY);
  rightWheel1->setPosition(INFINITY);
  rightWheel2->setPosition(INFINITY);
  leftWheel1->setVelocity(0.0);
  leftWheel2->setVelocity(0.0);
  rightWheel1->setVelocity(0.0);
  rightWheel2->setVelocity(0.0);
  
  while (robot->step(TIME_STEP) != -1) {
    double frontValue = frontSensor->getValue();
    double leftValue1 =  leftSensor1 ->getValue();
    double leftValue2 =  leftSensor2->getValue();
    double rightValue1 = rightSensor1->getValue();
    double rightValue2=  rightSensor2->getValue();
    std::cout<<leftValue1<<" "<<leftValue2<<" "<<frontValue<<" "<<rightValue1<<" "<<rightValue2<<std::endl;
   
   
    if (frontValue<=999){
   leftmotor_speed=5;
      rightmotor_speed=5;
      if((rightValue1)==(leftValue2)){
      
     if((rightValue2)>(leftValue2)){
     leftmotor_speed=30.0;
      rightmotor_speed=-30.0;
    
    }
    if((rightValue2)<(leftValue2)){
    
      leftmotor_speed=-30.0;
     rightmotor_speed=30.0;
    
    }
    }
    
    
    
    
    
    
     
     
   
if((rightValue1+rightValue2)>(leftValue1+leftValue2)){
        leftmotor_speed=30.0;
      rightmotor_speed=-30.0;
      }
      else if((rightValue1+rightValue2)<(leftValue1+leftValue2)){
      leftmotor_speed=-30.0;
     rightmotor_speed=30.0;
     }


   }
   
 
 
 
 
 
 else{
 
    error=(rightValue1-leftValue1)/80;
    //error2=(rightValue2-leftValue2);
    //error=(error1*2+error2*1.5)/80;
    
    integral+=error;
    derivative=(error-previous_error);
    PID_error=(Kp*error+Ki*integral+Kd*derivative);
    
    
  leftmotor_speed=speed+PID_error;
   rightmotor_speed=speed-PID_error;
   
 
 }
 leftWheel1->setVelocity(leftmotor_speed);
 leftWheel2->setVelocity(leftmotor_speed);
 rightWheel1->setVelocity(rightmotor_speed);
 rightWheel2->setVelocity(rightmotor_speed);
 std::cout<<leftmotor_speed<<" "<<rightmotor_speed<<" "<<PID_error<<std::endl;
 }
 }
      