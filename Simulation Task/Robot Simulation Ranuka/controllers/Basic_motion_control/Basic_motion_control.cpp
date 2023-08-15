//Team Name :- Team Spectro
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
//#include <webots/LED.hpp>
//#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 32
using namespace webots;
  Motor *wheels[4];
  DistanceSensor *DS[8]; 
  Motor *base; 
  Motor *camControl; 
  
  
  
  int counter = 0;
  int speed = 5;
  int sensorReadings[8];
  double lineControlSignal = 0.0;
  double lineErrorSum = 0.0;
  double lineError[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  
  double line_Kp = 1;
  double line_Kd = 1;
  double line_Ki = 0.01; 

  double lineErrorPredict = 0.0;
  //int x = 7;   // predicting sample
  //bool turning = false;
  //double maxSpeed = 5.0;
  
  double rotate = 0.0;
  double upAndDown = 0.0;

  
  
  void forward();
  void reverse();
  void turnRight();
  void turnLeft();
  void stop();
  void skidDrive(double Lspeed, double Rspeed);
  void readLineSensors();
  int  compareLines( double values);
  void pidLineFollower();
  double maxSpeedControl(double lineControlSignal);
  
  


int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Keyboard kb;
  
  Camera *cm;
  cm= robot->getCamera("cam");
  cm->enable(TIME_STEP);

  char wheels_names[4][15] = {"leftTopMotor", "leftRearMotor","rightTopMotor", "rightRearMotor"};                                //store names of motors
  char dsNames[8][8] = {"IR1","IR2","IR3","IR4","IR5","IR6","IR7","IR8"}; 
  
  for (int i = 0; i < 4; i++) {                                      //Initializing motors using a for loop
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    
  }
  
  base=robot->getMotor("cmBase");
  camControl=robot->getMotor("camMotor");
  
  //Initialize distance sensors
  for (int j = 0; j<8; j++){                                                    //Enabling distance sensors using for a loop
     DS[j] = robot->getDistanceSensor(dsNames[j]);                 
     DS[j]->enable(TIME_STEP);                                      
 }
 
 
   kb.enable(TIME_STEP);

  
  

   while (robot->step(TIME_STEP) != -1) {
             int key= kb.getKey();
            //speed = maxSpeedControl(lineControlSignal);
             
             
             
             skidDrive(speed+lineControlSignal,speed-lineControlSignal);
             counter+=1;
             readLineSensors();
             pidLineFollower();
          
             //std::cout <<"Color codes "<< DS[0]->getValue() <<"  " << DS[1]->getValue() <<"  "<< DS[2]->getValue() <<"  "<< DS[3]->getValue() <<"  "<< DS[4]->getValue() <<"  " <<"Front motors " <<DS[5]->getValue() <<"  " << DS[6]->getValue() <<"  " << DS[7]->getValue() <<std::endl;   
             //std::cout << sensorReadings[0] <<" " << sensorReadings[1] <<" " << sensorReadings[2] <<" " << sensorReadings[3] <<" " << sensorReadings[4] <<" " << sensorReadings[5] <<" " << sensorReadings[6] <<" " << sensorReadings[7] <<" "  <<std::endl;   
             //std::cout <<lineErrorPredict <<" "<< lineError[3] <<" " <<lineErrorSum <<std::endl;
            //std::cout << lineError[0] <<" " << lineError[1] <<" " << lineError[2] <<" " << lineError[3] <<" " << lineError[4] <<" " <<lineErrorSum<<std::endl;   
            //std::cout<<lineControlSignal<<std::endl;
            
            if(key == 65 && rotate <1.57){
              rotate += 0.1;
              }
            else if(key == 68 && rotate >-1.57){
              rotate += -0.1; 
              }
            else {
              rotate +=0; 
            }
            base->setPosition(rotate);
            
            
        
            
            if(key == 87 && upAndDown <1.57){
              upAndDown += 0.1;
              }
            else if(key == 83 && upAndDown >-1.57){
              upAndDown += -0.1; 
              }
            else {
              upAndDown +=0; 
            }
            camControl->setPosition(upAndDown);
            
            
            std::cout<<key<<std::endl;
            
            
            
            
            
            
            
            std::cout<<speed<<std::endl;
  }  
}

 void forward(){
               wheels[0]->setVelocity(speed);
               wheels[1]->setVelocity(speed);
               wheels[2]->setVelocity(speed);
               wheels[3]->setVelocity(speed);
 }
void reverse(){
               wheels[0]->setVelocity(-speed);
               wheels[1]->setVelocity(-speed);
               wheels[2]->setVelocity(-speed);
               wheels[3]->setVelocity(-speed);
 }
 
 void turnRight(){
               wheels[0]->setVelocity(speed);
               wheels[1]->setVelocity(speed);
               wheels[2]->setVelocity(-speed);
               wheels[3]->setVelocity(-speed);
 }

 void turnLeft(){
               wheels[0]->setVelocity(-speed);
               wheels[1]->setVelocity(-speed);
               wheels[2]->setVelocity(speed);
               wheels[3]->setVelocity(speed);
 }
 
 void stop(){
               wheels[0]->setVelocity(0);
               wheels[1]->setVelocity(0);
               wheels[2]->setVelocity(0);
               wheels[3]->setVelocity(0);
 }

 void skidDrive(double Lspeed , double Rspeed){                //Differential drive
               wheels[0]->setVelocity(Lspeed);
               wheels[1]->setVelocity(Lspeed);
               wheels[2]->setVelocity(Rspeed);
               wheels[3]->setVelocity(Rspeed);
 }
 
 void readLineSensors(){
               
                for (int j = 0; j <8; j++){
                   sensorReadings[j] = DS[j]->getValue();                                                    //Enabling distance sensors using for a loop
                   sensorReadings[j] = compareLines( sensorReadings[j]);              
                                              
                   }
 }
 
 int compareLines(double value){                    // Extracting colors
                if (value<100){
                  return 1;
                }
                else if((value>100) and (value <150)){
                  return 2;
                }
                else if(value>600 and value <700){
                  return 3;
                }
                else if(value>700){
                  return 0;
                }
                else {
                  return 5;
                }
 }

 /*double maxSpeedControl(double lineControlSignal){
                double speed = maxSpeed;
                if(lineControlSignal > 4){
                   speed = maxSpeed - lineControlSignal;
                }
                 return speed; 
 } */

 void pidLineFollower(){
                for (int i = 0; i<4 ; i++){                           // Storing 5 previous lineError values
                  lineError[i] = lineError[i+1];
                }

              
                if (lineError[3]>lineError[2]){                          
                  if (lineError[3]<lineError[2]+8){                      
                      
                  }
                  else{
                    lineError[3] = lineError[2];
                  }
                }
                else if(lineError[3]<lineError[2]){
                  if (8+lineError[3]>lineError[2]){                      
                      
                  }
                   else{
                    lineError[3] = lineError[2];
                  }
                }
                
                lineErrorSum += lineError[3];                    // Get sum of the errors
                

          
                //lineError[4] = 2*(sensorReadings[7]-sensorReadings[5])+2*(1-sensorReadings[7])*(1-sensorReadings[5])*(sensorReadings[4]-sensorReadings[0])+(sensorReadings[4]-sensorReadings[1]);      // Error algorithm
                lineError[4] = 3*(sensorReadings[0]-sensorReadings[7])+2*(sensorReadings[1]-sensorReadings[6])+(sensorReadings[2]-sensorReadings[5]);      // Error algorithm

               
                lineErrorPredict = (lineError[3]-lineError[1 ]);       // Derivative
               
                
                //lineErrorPredict = ((x-2)*(x-3)*(x-4)/(-6)*(lineError[0])) + ((x-1)*(x-3)*(x-4)/(2)*(lineError[1])) + ((x-1)*(x-2)*(x-4)/(-2)*(lineError[2])) + ((x-1)*(x-2)*(x-3)/(6)*(lineError[3]))/100;
                
                /////////////////////////////////    PID control signal  ////////////////////////////

                lineControlSignal = 0.8*lineError[4] + 0.5*line_Ki*lineErrorSum + lineErrorPredict;    

                ///////////////////////////////////////////////////////////////////////////////////  
                
                // Defining boundaries
                
                if ((lineErrorSum > 15) or (lineErrorSum < 15)){                      //Regulating the max speed
                     speed = 3;
                  }
                else if ((lineErrorPredict>8)){                      //Regulating the max speed
                     speed = 2;
                  }

                if (lineErrorSum>30){                               //Regulating max lineError
                  lineErrorSum = 30;
                } 
                else if (lineErrorSum<-30){                               //Regulating max lineError
                  lineErrorSum = -30;
                }
           

 }


 




