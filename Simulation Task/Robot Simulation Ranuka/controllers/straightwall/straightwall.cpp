#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // for std::find

using namespace webots;
using namespace std;

const int TIME_STEP = 16;
const double MAX_SPEED = 10.0;

// Motor names
const int NUM_MOTORS = 4;
const char *motorNames[] = {"right_front_wheel", "right_back_wheel","left_front_wheel", "left_back_wheel"};

//IR sensor names
const int NUM_IR_SENSORS = 6;
const char *irSensorNames[] = {"ir_ext_left", "ir_left", "ir_middle_left", "ir_middle_right", "ir_right", "ir_ext_right"};

// Sonar sensor names
const int NUM_SONAR_SENSORS = 4;  //4
const char *sonarSensorNames[] = {"sonar_left", "sonar_front_left" , "sonar_front_right", "sonar_right"}; //


double kp = 4;  //4
double ki = 0.2;  //0
double kd = 1; //1


std::vector<double> PID(double integral, double previousError, double *readings) {  //PID funtion return correction, sumErrors and Error
  double error = 0;
  double coefficients[6] = {-3000, -2000, -1000, 1000, 2000, 3000};
  for (int i = 0; i < 6; ++i) {
    error += coefficients[i] * readings[i];
  }

  double P = kp * error;
  double I = integral + (ki * error);
  double D = kd * (error - previousError);
  double correction = (P + I + D) / 1000.0;

  return {correction, integral, error};
}


int main(int argc, char **argv){
  double irIntegral = 0.0;
  double irPreviousError = 0.0;
  double sonarIntegral = 0.0;
  double sonarPreviousError = 0.0;

  Robot *robot = new Robot();

    // Initialize motors from right to left
  Motor *motors[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }


  ////////Sensors initialized from left to right///////
  // Initialize IR sensors
  DistanceSensor *irSensors[NUM_IR_SENSORS];
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
    irSensors[i]->enable(TIME_STEP);
  }

  // Initialize sonar sensors
  DistanceSensor *sonarSensors[NUM_SONAR_SENSORS];
  for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
    sonarSensors[i] = robot->getDistanceSensor(sonarSensorNames[i]);
    sonarSensors[i]->enable(TIME_STEP);
  }


  while (robot->step(TIME_STEP) != -1) {

    // Read IR sensor values
    double irSensorValues[NUM_IR_SENSORS];  //values are stored from right to left
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
      cout<<irSensors[i]->getValue()<<endl;
      if (irSensors[i]->getValue() > 512) {
        irSensorValues[i] = 0;
      } else if (irSensors[i]->getValue() < 180) {
        irSensorValues[i] = 1;
      }
    }
    
    double* irValueFound = std::find(irSensorValues, irSensorValues+6, 1);

    if (irValueFound != irSensorValues + 6) {
      auto result = PID(irIntegral, irPreviousError, irSensorValues);
      double correction = result[0];
      double irIntegral = result[1];
      double irPreviousError = result[2];

      double l_speed = 5.0 - correction;
      double r_speed = 5.0 + correction;
      if (l_speed < 0.0) {l_speed = 0.0;}
      if (l_speed > 10.0) {l_speed = 10.0;}
      if (r_speed < 0.0) {r_speed = 0.0;}
      if (r_speed > 10.0) {r_speed = 10.0;}
      motors[0]->setVelocity(r_speed);
      motors[1]->setVelocity(r_speed);
      motors[2]->setVelocity(l_speed);
      motors[3]->setVelocity(l_speed);
      cout<<"IR correction: ";
      cout<<correction<<endl;
    }

    else {
          // Read sonar sensor values
      double sonarSensorValues[NUM_SONAR_SENSORS+2];
      for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
        sonarSensorValues[i+1] = sonarSensors[i]->getValue();
        std::cout<<sonarSensors[i]->getValue();  // higher values mean more distance to an obstacle 
        std::cout<<'|';
      }
      std::cout<<' '<<std::endl;

      sonarSensorValues[0]=sonarSensorValues[5]=0;

      if (sonarSensorValues[2] > sonarSensorValues[3] && sonarSensorValues[1]> 400) {
        sonarSensorValues[1] = sonarSensorValues[2] =  1;
        sonarSensorValues[3] = sonarSensorValues[4] = 0;
      } else if (sonarSensorValues[2] > 900 && sonarSensorValues[3] > 900 && sonarSensorValues[1] > 500 && sonarSensorValues[4] > 500) {
        sonarSensorValues[1] = sonarSensorValues[4] = 0;
        sonarSensorValues[3] = sonarSensorValues[2] = 1;
      } else if (sonarSensorValues[3] > sonarSensorValues[2] || sonarSensorValues[1] < sonarSensorValues[4]) {
        sonarSensorValues[1] = sonarSensorValues[2] = 0;
        sonarSensorValues[3] = sonarSensorValues[4] = 1;
      }

      for (int i = 0; i < 6; ++i) {
        cout << sonarSensorValues[i] << " ";
      }
        cout << endl;
 
      auto result = PID(sonarIntegral, sonarPreviousError, sonarSensorValues);
      double correction = result[0]; 
      double sonarIntegral = result[1];
      double sonarPreviousError = result[2];

      double l_speed = 5.0 - correction; //5
      double r_speed = 5.0 + correction;
      if (l_speed < 0.0) {l_speed = 0.0;}
      if (l_speed > 10.0) {l_speed = 10.0;} //10
      if (r_speed < 0.0) {r_speed = 0.0;}
      if (r_speed > 10.0) {r_speed = 10.0;}
      motors[0]->setVelocity(r_speed);
      motors[1]->setVelocity(r_speed);
      motors[2]->setVelocity(l_speed);
      motors[3]->setVelocity(l_speed);
     cout<<"Sonar Correction: ";
     cout<<correction<<endl;

      
    }
}
delete robot;
return 0;
}
