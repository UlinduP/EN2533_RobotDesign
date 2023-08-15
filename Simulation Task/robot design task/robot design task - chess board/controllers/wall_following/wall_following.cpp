#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

#define TIME_STEP 16
#define TARGET_SPEED 2.0

// Helper function to map a value from one range to another
double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double pidController(double error, double lastError, double integral, double kp, double ki, double kd) { //integral stands for the accumulated errors up to that point
  double derivative = error - lastError;
  integral += error;
  double output = kp * error + ki * integral + kd * derivative;
  return output;
}

double sonarError(double *sonarSensorValues, int numSonarSensors) { // sonar sensors in the right side needed to be addded first to the array
  // Compute the error as the difference between the right and left sensors
  //double right = 0.0;
  //double left = 0.0;

  double error = 0.0;
  double sonar_weight[4] = {-0.5, -0.2, 0.2, 0.5};

  if ((sonarSensorValues[2]>sonarSensorValues[1]) && (sonarSensorValues[3]>0.4)){
    sonarSensorValues[3]=1;   //1->3, 2->2, 3->1, 4->0
    sonarSensorValues[2]=1;
    sonarSensorValues[1]=0;
    sonarSensorValues[0]=0;
  }
  else if ((sonarSensorValues[2]>0.9) && (sonarSensorValues[1]>0.9) && (sonarSensorValues[3]>0.5) && (sonarSensorValues[0]>0.5)){
    sonarSensorValues[3]=0;
    sonarSensorValues[0]=0;
    sonarSensorValues[1]=1;
    sonarSensorValues[2]=1;
  }

  else if ((sonarSensorValues[1]>sonarSensorValues[2]) || (sonarSensorValues[3]<sonarSensorValues[0])){
    sonarSensorValues[3]=0;
    sonarSensorValues[2]=0;
    sonarSensorValues[1]=1;
    sonarSensorValues[0]=1;
  }

  // Compute weighted average of IR sensor values
  for (int i = 0; i < numSonarSensors; i++) {              
    error += sonar_weight[i] * sonarSensorValues[i];
  }
 
  return error;  //range [-1.7,1.7]
}


  //for (int i = 0; i < numSonarSensors; i++) {
   // if (i < numSonarSensors/2) {
   //   right += sonarValues[i];
   // } else {
  //    left += sonarValues[i];
   // }
 // }
  //right /= numSonarSensors/2;
 // left /= numSonarSensors/2;
  //double error = right - left;

  // Map the error to a range of -1 to 1
  //double maxError = 2.0;
  //error = map(error, -maxError, maxError, -1.0, 1.0);
  //std::cout<<"Sonar Error: ";

void updateMotorSpeeds(double speed, Motor **motors) {
  // Map the output value from the PID controller to a motor speed
  //double speed = map(output, -1.0, 1.0, -MAX_SPEED, MAX_SPEED);
  //std::cout<<speed<<std::endl;
 
  // Update the motor speeds
  motors[0]->setVelocity(TARGET_SPEED+speed);
  motors[1]->setVelocity(TARGET_SPEED+speed);
  motors[2]->setVelocity(TARGET_SPEED-speed);
  motors[3]->setVelocity(TARGET_SPEED-speed);
}


// Motor names
const int NUM_MOTORS = 4;
const char *motorNames[] = {"right_front_wheel", "right_back_wheel","left_front_wheel", "left_back_wheel"};


// Sonar sensor names
const int NUM_SONAR_SENSORS = 4;  //4
const char *sonarSensorNames[] = {"sonar_right", "sonar_front_right" , "sonar_front_left", "sonar_left"}; //

//variables for PID controller
double lastErrorSonar = 0.0;
double integralSonar = 0.0;

int main(int argc, char **argv){

    // Create the robot object
  Robot *robot = new Robot();

  
  // Initialize motors
  Motor *motors[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }


    // Initialize sonar sensors
  DistanceSensor *sonarSensors[NUM_SONAR_SENSORS];
  for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
    sonarSensors[i] = robot->getDistanceSensor(sonarSensorNames[i]);
    sonarSensors[i]->enable(TIME_STEP);
  }

    // Main control loop
  while (robot->step(TIME_STEP) != -1) {

    // Read sonar sensor values
    double sonarSensorValues[NUM_SONAR_SENSORS];
    for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
      sonarSensorValues[i] = sonarSensors[i]->getValue()/1000;
      std::cout<<sonarSensorValues[i]<<std::endl;  // higher values mean more distance to an obstacle 
    }

      // Read Sonar sensor values and compute error
    double sonarerror = sonarError(sonarSensorValues, NUM_SONAR_SENSORS);

    // Compute control signal using PID controller
    double output = pidController(sonarerror, lastErrorSonar, integralSonar, 1.5, 0.01, 0);
      
    // Update motor speeds based on control signal
    updateMotorSpeeds(output, motors);
      
    // Update PID variables for next time step
    lastErrorSonar = sonarerror;
    integralSonar += sonarerror;
  }

  // Cleanup
  delete robot;
  return 0;
}
