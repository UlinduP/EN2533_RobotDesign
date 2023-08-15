#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;

#define TIME_STEP 16
#define TARGET_SPEED 2.0

double pidController(double error, double lastError, double integral, double kp, double ki, double kd) { //integral stands for the accumulated errors up to that point
  double derivative = error - lastError;  // here error [-4.5,4.5]
  integral += error;
  double output = kp * error + ki * integral + kd * derivative;
  return output;
}

double irError(double *irSensorValues, int numIRSensors){
  double error = 0.0;
  double ir_weight[6] = {-1.0, -0.5, -0.2, 0.2, 0.5, 1.0};

  // Compute weighted average of IR sensor values
  for (int i = 0; i < numIRSensors; i++) {              
    error += ir_weight[i] * irSensorValues[i];
  }
 
  return error;  //range [-1.7,1.7]
}

void updateMotorSpeeds(double speed, Motor **motors){
  // Map the output value from the PID controller to a motor speed
  //double speed = map(output, -1.0, 1.0, -MAX_SPEED, MAX_SPEED);
  //std::cout<<speed<<std::endl;
 
  // Update the motor speeds
  motors[0]->setVelocity(TARGET_SPEED+speed);
  motors[1]->setVelocity(TARGET_SPEED+speed);//2 selected arbitrarily. Lesser speeds the better
  motors[2]->setVelocity(TARGET_SPEED-speed);
  motors[3]->setVelocity(TARGET_SPEED-speed);
}


// Motor names
const int NUM_MOTORS = 4;
const char *motorNames[] = {"right_front_wheel", "right_back_wheel","left_front_wheel", "left_back_wheel"};


//IR sensor names
const int NUM_IR_SENSORS = 6;
const char *irSensorNames[] = {"ir_ext_right", "ir_right", "ir_middle_right", "ir_middle_left", "ir_left", "ir_ext_left"};


// Variables for PID controller
double lastErrorIr = 0.0;
double integralIr = 0.0;

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

  // Initialize IR sensors
  DistanceSensor *irSensors[NUM_IR_SENSORS];
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
    irSensors[i]->enable(TIME_STEP);
  }

  // Main control loop
  while (robot->step(TIME_STEP) != -1) {

    // Read IR sensor values
    double irSensorValues[NUM_IR_SENSORS];  //values are stored from right to left
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
      irSensorValues[i] = irSensors[i]->getValue()/1000;  // ir sensor values are between 0 and 1 with higher values (Lower values in here though) indicating the presence of the line.
      std::cout<<irSensorValues[i]<<std::endl; 
    }
    
      // Compute IR error
      double irerror = irError(irSensorValues, NUM_IR_SENSORS);

      // Compute control signal using PID controller
      double output = pidController(irerror, lastErrorIr, integralIr, 10, 0, 0);
      
      // Update motor speeds based on control signal
      updateMotorSpeeds(output, motors);
      
      // Update PID variables for next time step
      lastErrorIr = irerror;
      integralIr += irerror;
    }

  // Cleanup
  delete robot;
  return 0;
}