#include <string>
#include <vector>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#define MAX_SPEED 7
#define MAX_SPEED_line_follow 5
#define NOM_SPEED_MAZE 4
#define MAX_SPEED_MAZE 9
#define TIME_STEP 16
#define WHEEL_RADIUS 0.025

using namespace webots;
using namespace std;

int moveForward(float distance);
double moveForwardDist();
float getBearing(double x, double z);
void turnRobot(float angle);
std::vector<double> PID(double integral, double previousError, double *readings) ;
void lineFollow();
void wallFollow();
void detectKing(double rotation);
void goToKing();
void detectObjectBeforeKing(double angle);
void sensorUp();
void sensorDown();
int checkPath(double v_angle, double j_angle, double r_angle);
void placeRook();
void pickRook();
void chessBoard();
void kingPos();
int untillWallFollow();
int walltoChess();
void turn();
void stop();
void movetoRook();


double bearing;
int turnInProcess;

// Motor names
const int NUM_MOTORS = 4;
const char *motorNames[] = {"right_front_wheel", "right_back_wheel","left_front_wheel", "left_back_wheel"};

//IR sensor names
const int NUM_IR_SENSORS = 8;
double irSensorValues[NUM_IR_SENSORS];  //values are stored from right to left
const char *irSensorNames[] = {"ir_ext_ext_left","ir_ext_left", "ir_left", "ir_middle_left", "ir_middle_right", "ir_right", "ir_ext_right","ir_ext_ext_right"};

// Sonar sensor names
const int NUM_SONAR_SENSORS = 4;  //4
double sonarSensorValues[NUM_SONAR_SENSORS+4];
const char *sonarSensorNames[] = {"sonar_left", "sonar_front_left" , "sonar_front_right", "sonar_right"}; //


double kp = 8;  //4
double ki = 0.2;  //0
double kd = 1; //1
double irIntegral = 0.0;
double irPreviousError = 0.0;
double sonarIntegral = 0.0;
double sonarPreviousError = 0.0;
bool check_white = false;
const double *lidar_values;
double king_angle = 0;  // 0  to -3.14 rad
double king_row_distance;
double laser_dist;
bool king_row_found=false;
bool success=false;
double distance_travelled_towards_king;
double kingAngle[2];
string dotted_line_color = "red";
double ir[8];
  


Robot *robot = new Robot();
Motor *motors[4];
DistanceSensor *irSensors[8];
DistanceSensor *sonarSensors[4];
DistanceSensor *king_color_detector;
Motor *rotational_motor;
PositionSensor *rotational_sensor;
Motor *linear_motor;
DistanceSensor *laser_sensor;
Compass *compass;
PositionSensor *right_front_wheel_pos_sensor;




int main(int argc, char **argv){
  
  cout<<5<<endl;
  // Initialize motors from right to left
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  ////////Sensors initialized from left to right///////
  // Initialize IR sensors
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    irSensors[i] = robot->getDistanceSensor(irSensorNames[i]);
    irSensors[i]->enable(TIME_STEP);
  }

  // Initialize sonar sensors
  for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
    sonarSensors[i] = robot->getDistanceSensor(sonarSensorNames[i]);
    sonarSensors[i]->enable(TIME_STEP);
  }



  rotational_motor = robot->getMotor("rotational_motor");
 // rotational_motor->setPosition(0.0);
  rotational_sensor = robot->getPositionSensor("rotational_sensor");
  rotational_sensor->enable(TIME_STEP);
//
  linear_motor = robot->getMotor("linear_motor");
  linear_motor->setPosition(0.6);
//
    king_color_detector=robot->getDistanceSensor("king_color_detector");
    king_color_detector->enable(TIME_STEP);

    laser_sensor = robot->getDistanceSensor("laser_sensor");
    laser_sensor->enable(TIME_STEP);


  // Initializing the compass
  compass = robot->getCompass("compass");
  compass->enable(TIME_STEP);
  //robot->step(100);

  // initializing position sensors
  right_front_wheel_pos_sensor = robot->getPositionSensor("right_front_wheel_pos_sensor");
  right_front_wheel_pos_sensor->enable(TIME_STEP);
  //robot->step(100);
  

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(TIME_STEP) != -1)
  {
      //untillWallFollow();
      //kingPos();
      //cout<<kingAngle[0]<<endl;
      //turnRobot(0);

      //chessBoard();
      cout<<"Simulation Started!"<<endl;
      if (untillWallFollow()){
        moveForward(5);
        if (walltoChess){
          movetoRook();
          //pickRook();
          chessBoard();
        }
      }

      delete robot;
      return 0;
    // while (robot->step(TIME_STEP) != -1){
    // }
  } // end of main while loop
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

int untillWallFollow()
{
  bool wall_follow = false;
  bool line_follow = false;
  while (robot->step(TIME_STEP) != -1)
    {
      for (int i = 0; i < NUM_IR_SENSORS; i++) {
      cout<<irSensors[i]->getValue()<<'-';
      if (irSensors[i]->getValue() > 512) {
        irSensorValues[i] = 0;
      } else if (irSensors[i]->getValue() < 250) {
        check_white = true;
        irSensorValues[i] = 1;
      }
      cout<<irSensorValues[i]<<'|';
      }
      cout<<' '<<endl;

      if (check_white)
      {
        lineFollow();
        if (wall_follow)
        {
          line_follow = true;
        }
      }
      else
      {
        wallFollow();
        wall_follow = true;
      }

      check_white = false;

      if (wall_follow && line_follow)
      {
        return 1;
      }
    
    }
  return 0;
}

void turn()
{
  if (ir[0]+ir[1]+ir[2] > ir[7]+ir[6]+ir[5] ){
    cout<<"Blue is in left"<<endl;
    if (dotted_line_color == "red"){
      turnRobot(90);
    }else {turnRobot(-90);}
  }else{
    cout<<"Red is in left"<<endl;
    if (dotted_line_color == "red"){
      turnRobot(-90);
    }else{turnRobot(90);}
  }
}

void stop()
{
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      motors[2]->setVelocity(0);
      motors[3]->setVelocity(0);
}


int walltoChess()
{   
    int times = 2;
    while (robot->step(TIME_STEP) != -1)
    {
      int one = 0;
      for (int i = 0; i < NUM_IR_SENSORS; i++) {
      cout<<irSensors[i]->getValue()<<'-';
      ir[i] = irSensors[i]->getValue();
      if (irSensors[i]->getValue() > 512) {
        irSensorValues[i] = 0;
      } else if (irSensors[i]->getValue() < 250) {
        irSensorValues[i] = 1;
        one+=1;
      }
      cout<<irSensorValues[i]<<'|';
      }
      cout<<" "<<endl;
      lineFollow();

      if (one>=3)
      {
        stop();
        //robot->step(100);
        moveForward(3);
        turn();
        times-=1;
        if (times == 0){return 1;}
      }
    }return 0;
}


int moveForward(float distance = -1)  // 0.75 in scale is 15 in sensor val
{

  // NOTE: Here speed is a fraction. It implies the fraction of maximum speed
  float startPos = right_front_wheel_pos_sensor -> getValue();
  // set the motors' velocity to move forward
  motors[0]->setVelocity(MAX_SPEED);
  motors[1]->setVelocity(MAX_SPEED);
  motors[2]->setVelocity(MAX_SPEED);
  motors[3]->setVelocity(MAX_SPEED);

  if (distance > 0){
    while (robot->step(TIME_STEP) != -1){
      //cout<<right_front_wheel_pos_sensor -> getValue()<<endl;
      if(abs(right_front_wheel_pos_sensor -> getValue() - startPos) > distance){
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      motors[2]->setVelocity(0);
      motors[3]->setVelocity(0);
        break;
      }
    }
  }

  return 0;
} // end of moveForward function

double moveForwardDist()  // 0.75 in scale is 15 in sensor val
{

  // NOTE: Here speed is a fraction. It implies the fraction of maximum speed
  double startPos = right_front_wheel_pos_sensor -> getValue();

  while (robot->step(TIME_STEP) != -1){
    if (laser_sensor->getValue() > 50)
    {
      motors[0]->setVelocity(MAX_SPEED);
      motors[1]->setVelocity(MAX_SPEED);
      motors[2]->setVelocity(MAX_SPEED);
      motors[3]->setVelocity(MAX_SPEED);
    }
    
    else
    {
      motors[0]->setVelocity(0.0);
      motors[1]->setVelocity(0.0);
      motors[2]->setVelocity(0.0);
      motors[3]->setVelocity(0.0);
      //robot->step(1000);
      return (right_front_wheel_pos_sensor->getValue()-startPos);
    }

    } // end of moveForward function
  return 0;
}

void movetoRook()
{
 // NOTE: Here speed is a fraction. It implies the fraction of maximum speed

  while (robot->step(TIME_STEP) != -1){
    if (sonarSensors[1]->getValue() > 50 && sonarSensors[2]->getValue() > 50)
    {
      motors[0]->setVelocity(MAX_SPEED);
      motors[1]->setVelocity(MAX_SPEED);
      motors[2]->setVelocity(MAX_SPEED);
      motors[3]->setVelocity(MAX_SPEED);
    }
    
    else
    {
      motors[0]->setVelocity(0.0);
      motors[1]->setVelocity(0.0);
      motors[2]->setVelocity(0.0);
      motors[3]->setVelocity(0.0);
      break;
    }
}
}

float getBearing(double x, double z)
{
  // x  --> sin curve
  // z  --> cos curve

  x = compass->getValues()[0];
  z = compass->getValues()[2];

  bearing = asin(x) * 57.29577;

  if (x > z && z < 0)
  {
    bearing = 180 - bearing;
  }
  else if (z < 0)
  {
    // bot in 225,360   0,45
    bearing = -180 - bearing;
  }
  if (bearing < 0)
  {
    bearing = 360 + bearing;
  }

  return bearing;
}

void turnRobot(float angle) //angle in degrees
{
  // reading compass direction
  double x = compass->getValues()[0];
  double z = compass->getValues()[2];

  double start_location = getBearing(x, z);

  double end_location = start_location + angle;
  double diff{};
  double I{};

  // if (end_location < 0) {end_location += 360; status = true;}
  // if (end_location > 360) {end_location -= 360; status = true;}
  if (end_location < 0)
  {
    end_location += 360;
  }
  if (end_location > 360)
  {
    end_location -= 360;
  }

  motors[0]->setVelocity((angle / abs(angle)) * 0.6 * MAX_SPEED);
  motors[1]->setVelocity((angle / abs(angle)) * 0.6 * MAX_SPEED);
  motors[2]->setVelocity((angle / abs(angle)) * (-0.6) * MAX_SPEED);
  motors[3]->setVelocity((angle / abs(angle)) * (-0.6) * MAX_SPEED);

  while (robot->step(TIME_STEP) != -1)
  {

    x = compass->getValues()[0];
    z = compass->getValues()[2];

    bearing = getBearing(x, z);

    diff = abs(bearing - start_location);
    if (diff > 2.27198)
      diff = 2.27198;

    I += diff;
    if (I > abs(angle))
    {
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      motors[2]->setVelocity(0);
      motors[3]->setVelocity(0);
      break;
    }
    start_location = bearing;
  } // end of while loop
  turnInProcess = 0;

} // end of turning 

std::vector<double> PID(double integral, double previousError, double *readings) {  //PID funtion return correction, sumErrors and Error
  double error = 0;
  double coefficients[8] = {-16 ,-8, -4, -2, 2, 4, 8, 16};
  for (int i = 0; i < 8; ++i) {
    error += coefficients[i] * readings[i];
  }

  double P = kp * error;
  double I = integral + (ki * error);
  double D = kd * (error - previousError);
  double correction = (P + I + D)/30;

  return {correction, integral, error};
}

void lineFollow()
{
      auto result = PID(irIntegral, irPreviousError, irSensorValues);
      double correction = result[0];
      double irIntegral = result[1];
      double irPreviousError = result[2];

      double l_speed = 2.5 - correction;
      double r_speed = 2.5 + correction;
      if (l_speed < 0.0) {l_speed = 0.0;}
      if (l_speed > 5.0) {l_speed = 5.0;}
      if (r_speed < 0.0) {r_speed = 0.0;}
      if (r_speed > 5.0) {r_speed = 5.0;}
      motors[0]->setVelocity(r_speed);
      motors[1]->setVelocity(r_speed);
      motors[2]->setVelocity(l_speed);
      motors[3]->setVelocity(l_speed);
      cout<<"IR correction: ";
      cout<<correction<<endl;
}

void wallFollow()
{
                  // Read sonar sensor values
    for (int i = 0; i < NUM_SONAR_SENSORS; i++) {
      sonarSensorValues[i+2] = sonarSensors[i]->getValue();
      std::cout<<sonarSensors[i]->getValue();  // higher values mean more distance to an obstacle 
      std::cout<<'|';
    }
      std::cout<<' '<<std::endl;

    sonarSensorValues[0]=sonarSensorValues[6]=sonarSensorValues[1]=sonarSensorValues[7]=0;

    if (sonarSensorValues[3] > sonarSensorValues[4] && sonarSensorValues[2]> 400) {
      sonarSensorValues[2] = sonarSensorValues[3] = 1;
      sonarSensorValues[4] = sonarSensorValues[5] = 0;
    } else if (sonarSensorValues[3] > 900 && sonarSensorValues[4] > 900 && sonarSensorValues[2] > 500 && sonarSensorValues[5] > 500) {
      sonarSensorValues[2] = sonarSensorValues[5] = 0;
      sonarSensorValues[4] = sonarSensorValues[3] = 1;
    } else if (sonarSensorValues[4] > sonarSensorValues[3] || sonarSensorValues[2] < sonarSensorValues[5]) {
      sonarSensorValues[2] = sonarSensorValues[3] = 0;
      sonarSensorValues[4] = sonarSensorValues[5] = 1;
    }

    for (int i = 0; i < 8; ++i) {
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

void detectKing(double rotation)
 {
  while (robot->step(TIME_STEP) != -1) {
    if (rotation<0){
      rotational_motor->setVelocity(0.0);
      break;
    }
    rotational_motor->setPosition(rotation);
    if (laser_sensor->getValue()<1000)
    {
      king_angle = rotational_sensor->getValue();
      cout<<"King Angle: "<<king_angle<<endl;
      laser_dist = laser_sensor->getValue();
      cout<<"Laser Sensor value: "<<laser_dist<<endl;
      break; 
    }  

    cout<<rotational_sensor->getValue()<<'-';
    cout<<laser_sensor->getValue()<<endl;
    rotation+=0.01;
  }
 }

void goToKing(double angle)
{
  double start_pos = right_front_wheel_pos_sensor -> getValue();
  while (robot->step(TIME_STEP) != -1)
  {
    //cout<<right_front_wheel_pos_sensor -> getValue()<<endl;
    //cout<<laser_sensor->getValue()<<endl;
    if (laser_sensor->getValue()<1000)
    {
      cout<<"King row found!"<<endl;
      king_row_distance = laser_sensor->getValue();
      cout<<"King row distance: "<<king_row_distance<<endl;
      king_row_found=true;;
      distance_travelled_towards_king = right_front_wheel_pos_sensor -> getValue() - start_pos;
      break;
    }
    if (sonarSensors[1]->getValue()<100 || sonarSensors[2]->getValue()<100)
    {
      cout<<"Chess piece detected! Wrong path!"<<endl;
      distance_travelled_towards_king = right_front_wheel_pos_sensor -> getValue() - start_pos;
      king_row_found = false;      
      break;
    }
    if(abs(right_front_wheel_pos_sensor -> getValue() - start_pos) > 6.5*15)  //6.5 is the number of squares
    {
      cout<<"Robot has gone outside the chess board"<<endl;
      distance_travelled_towards_king = right_front_wheel_pos_sensor -> getValue() - start_pos;
      break;
    }
    else
    {
      moveForward(0.5);
    }    
    
  }
}
 
 
void sensorUp()
{  
  
  while (robot->step(TIME_STEP) != -1) 
   {  
     double linear=0;
     cout<<"Laser sensor:"<<linear<<endl;
     linear+=0.005;
      
     if (linear>=0.53)
     {
       linear_motor->setVelocity(0.0);
       break;
     }  
     linear_motor->setPosition(linear);
      
    }
} 

void detectObjectBeforeKing(double angle)
{ 
  bool temp = false;
  double linear=0.6;
  while (robot->step(TIME_STEP) != -1) 
  {
    linear_motor->setPosition(linear);
    if (linear<=0.1)
    {
      linear_motor->setVelocity(0.0);
      temp = true;
      break;
    }  
    if (king_row_distance - laser_sensor->getValue() > 30)
    { 
      cout<<"Obstacle distance: "<<laser_sensor->getValue()<<endl;
      temp = false;
      break;
    }
    linear-=0.001;
  }
  
  if (temp)
  {
    cout<<"Possible Place!"<<endl;
    //boxDown();
    moveForwardDist();
    turnRobot(angle);
    moveForward(distance_travelled_towards_king);
    success = true;
  }

  else
  {
    cout<<"Obstacle in the row detected! Wrong path!"<<endl;
    turnRobot(angle);
    moveForward(distance_travelled_towards_king);
    success = false;
    robot->step(500);
  }

}

void kingPos()
{
    linear_motor->setPosition(0.6);
    robot->step(1000);
    moveForward(7.5);
    double rotation = M_PI;

    int i = 0;
    while (robot->step(TIME_STEP) != -1) {
        if (rotation>2*M_PI){
            //rotational_motor->setVelocity(0.0);
            //rotational_motor->setPosition(3*M_PI/2);
            //robot->step(500);
            break;
        }
        
        rotational_motor->setPosition(rotation);
        if (laser_sensor->getValue()<1000)
        {
            double king_angle = rotational_sensor->getValue() - M_PI;
            double laser_dist = laser_sensor->getValue();
            //checkPos.push
            kingAngle[i]=king_angle;

            i++;
         
        }  
    rotation+=0.01;
  }
}
void vZero()
{
    motors[0]->setVelocity(0);
    motors[1]->setVelocity(0);
    motors[2]->setVelocity(0);
    motors[3]->setVelocity(0);
}


int checkPath(double v_angle, double j_angle, double r_angle)  // angle the vehicle, junction and the motor needs to be rotated respectively
{ 
  double dist_junct;
  double king_row_distance;
  turnRobot(v_angle);
  robot->step(500);
  double start_pos = right_front_wheel_pos_sensor -> getValue();
  rotational_motor->setPosition(r_angle);
  robot->step(500);

  while (robot->step(TIME_STEP) != -1)
  {
    if (laser_sensor->getValue()<1000)
    {
      cout<<"King row found!"<<endl;
      king_row_distance = laser_sensor->getValue();
      cout<<"King row distance: "<<king_row_distance<<endl;
      dist_junct = right_front_wheel_pos_sensor -> getValue() - start_pos;

      turnRobot(j_angle);
      robot->step(500);
      rotational_motor->setPosition(r_angle+j_angle/180*M_PI);
      robot->step(1000);

      double linear=0.6;
      while (robot->step(TIME_STEP) != -1) 
      {
        linear_motor->setPosition(linear);
        if (linear<=0.1)
        {
          linear_motor->setVelocity(0.0);
          linear_motor->setPosition(0.6);

          double dist_king = moveForwardDist();

          if (king_color_detector->getValue() < 512)
          {
            cout<<"White king detected!"<<endl;
            turnRobot(180);
            moveForward(dist_king);
            //placeRook();
            turnRobot(-j_angle);
            moveForward(dist_junct);
            turnRobot(v_angle);
            linear_motor->setPosition(0.6);
            vZero();
            return 1;
          }

          else{
            cout<<"Black king detected!"<<endl;
            turnRobot(180);
            moveForward(dist_king);

            //going in the same path
            turnRobot(j_angle);
            linear_motor->setPosition(0.6);
            robot->step(500);
            rotational_motor->setPosition(r_angle);
            double junct_pos = right_front_wheel_pos_sensor->getValue();
            robot->step(500);
            moveForward(15);

            while (robot->step(TIME_STEP) != -1)
            {
              if (laser_sensor->getValue()<1000)
              {
                cout<<"King row found!"<<endl;
                king_row_distance = laser_sensor->getValue();
                cout<<"King row distance: "<<king_row_distance<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;

                turnRobot(j_angle);
                robot->step(500);
                rotational_motor->setPosition(r_angle+j_angle/180*M_PI);
                robot->step(1000);

                double linear = 0.6;
                while (robot->step(TIME_STEP)!=-1)
                {
                  linear_motor->setPosition(linear);
                  if (linear<=0.1)
                  {
                    linear_motor->setVelocity(0.0);
                    linear_motor->setPosition(0.6);
                    cout<<"White king detected!"<<endl;
                    //placeRook();
                    turnRobot(j_angle);
                    moveForward(dist_junct+dist_junct2);
                    turnRobot(v_angle);
                    linear_motor->setPosition(0.6);
                    vZero();
                    return 1;

                  }
                  if (king_row_distance - laser_sensor->getValue() > 30)
                  {
                    cout<<"Obstacle deistance: "<<laser_sensor->getValue()<<endl;
                    cout<<"Obstacle Detected!"<<endl;
                    turnRobot(j_angle-2);
                    moveForward(dist_junct+dist_junct2);
                    turnRobot(v_angle);
                    linear_motor->setPosition(0.6);
                    vZero();
                    return 0;
                  }linear-=0.001;
                }
              }else if (sonarSensors[1]->getValue()<100 || sonarSensors[2]->getValue()<100)
              {
                cout<<"Chess piece detected! Wrong path!"<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;
                turnRobot(180);
                moveForward(dist_junct+dist_junct2);
                turnRobot(v_angle);
                linear_motor->setPosition(0.6);
                vZero();
                return 0;
              }
              else if(abs(right_front_wheel_pos_sensor -> getValue() - junct_pos + dist_junct) > 6.5*15-5)  //6.5 is the number of squares
              {
                cout<<"Robot has gone outside the chess board"<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;
                turnRobot(180);
                moveForward(dist_junct+dist_junct2);
                turnRobot(v_angle);
                linear_motor->setPosition(0.6);
                vZero();
                return 0;
              }
              else
              {
                moveForward(0.5);
              }
              

          }
      
        }  
        }
        if (king_row_distance - laser_sensor->getValue() > 30)
        { 
          cout<<"Obstacle distance: "<<laser_sensor->getValue()<<endl;
          cout<<"Obstacle detected!"<<endl;
          turnRobot(-j_angle);
          linear_motor->setPosition(0.6);

           //going in the same path
            robot->step(1000);
            //robot->step(10000);
            rotational_motor->setPosition(r_angle);
            robot->step(2000);
            //robot->step(10000);
            double junct_pos = right_front_wheel_pos_sensor->getValue();
            moveForward(15);
            robot->step(500);

            while (robot->step(TIME_STEP) != -1)
            {
              if (laser_sensor->getValue()<1000)
              {
                cout<<"King row found!"<<endl;
                king_row_distance = laser_sensor->getValue();
                cout<<"King row distance: "<<king_row_distance<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;

                turnRobot(j_angle-2);
                robot->step(500);
                rotational_motor->setPosition(r_angle+j_angle/180*M_PI);
                robot->step(1000);

                double linear = 0.6;
                while (robot->step(TIME_STEP)!=-1)
                {
                  linear_motor->setPosition(linear);
                  if (linear<=0.1)
                  {
                    linear_motor->setVelocity(0.0);
                    linear_motor->setPosition(0.6);
                    cout<<"White king detected!"<<endl;
                    //placeRook();
                    turnRobot(j_angle-2);
                    moveForward(dist_junct+dist_junct2);
                    turnRobot(v_angle);
                    linear_motor->setPosition(0.6);
                    vZero();
                    return 1;

                  }
                  if (king_row_distance - laser_sensor->getValue() > 30)
                  {
                    cout<<"Obstacle deistance: "<<laser_sensor->getValue()<<endl;
                    cout<<"Obstacle Detected!"<<endl;
                    turnRobot(j_angle-2);
                    moveForward(dist_junct+dist_junct2);
                    turnRobot(v_angle);
                    linear_motor->setPosition(0.6);
                    vZero();
                    return 0;
                  }linear-=0.001;
                }
              }else if (sonarSensors[1]->getValue()<100 || sonarSensors[2]->getValue()<100)
              {
                cout<<"Chess piece detected! Wrong path!"<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;
                turnRobot(180);
                moveForward(dist_junct+dist_junct2);
                turnRobot(v_angle);
                linear_motor->setPosition(0.6);
                vZero();
                return 0;
              }
              else if(abs(right_front_wheel_pos_sensor -> getValue() - junct_pos + dist_junct) > 6.5*15-5)  //6.5 is the number of squares
              {
                cout<<"Robot has gone outside the chess board"<<endl;
                double dist_junct2 = right_front_wheel_pos_sensor -> getValue() - junct_pos;
                turnRobot(178);  //180
                moveForward(dist_junct+dist_junct2);
                turnRobot(v_angle);
                linear_motor->setPosition(0.6);
                vZero();
                return 0;
              }
              else
              {
                moveForward(0.5);
              }
            } 
        } linear-=0.001;
   
      }

    }
    else if (sonarSensors[1]->getValue()<100 || sonarSensors[2]->getValue()<100)
    {
      cout<<"Chess piece detected! Wrong path!"<<endl;
      dist_junct = right_front_wheel_pos_sensor -> getValue() - start_pos;
      turnRobot(180);
      moveForward(dist_junct);
      turnRobot(v_angle);
      linear_motor->setPosition(0.6);
      vZero();
      return 0;
    }
    else if(abs(right_front_wheel_pos_sensor -> getValue() - start_pos) > 6.5*15-5)  //6.5 is the number of squares
    {
      cout<<"Robot has gone outside the chess board"<<endl;
      dist_junct = right_front_wheel_pos_sensor -> getValue() - start_pos;
      turnRobot(180);
      moveForward(dist_junct);
      turnRobot(v_angle);
      linear_motor->setPosition(0.6);
      vZero();
      return 0;
    }
    else
    {
      moveForward(0.5);
    }    
    
  }  return 0;

}



void placeRook()
{
  
}

void chessBoard()


{
    for (int i=0;i<2;i++)
      {
        if (kingAngle[i]<M_PI/2)
        {
          if (checkPath(90,-90,0)){
            "Chessboard Done!";
            break;
          }
          else if (checkPath(0,90,M_PI)){
            "Chessboard Done!";
            break;
          }
        }
        else if (kingAngle[i]<M_PI){
          if (checkPath(-90,90,0)){
            "Chessboard Done!";
            break;
          }
          else if (checkPath(0,-90,0)){
            "Chessboard Done!";
            break;
          }

        }
      }

}






// Grabbing and lifting code

// #include <string>
// #include <vector>
// #include <math.h>
// #include <webots/Robot.hpp>
// #include <webots/Motor.hpp>
// #include <webots/DistanceSensor.hpp>
// #include <webots/PositionSensor.hpp>
// #include <webots/InertialUnit.hpp>
// #include <webots/Motor.hpp>
// #include <webots/utils/AnsiCodes.hpp>
// #include <iostream>

// using namespace webots;
// using namespace std;
// void turnArm(float angle);
// void PID();
// const char *wheel_names[4] = {"right_front_wheel", "left_front_wheel", "right_rear_wheel", "left_rear_wheel"};

// static const int TIME_STEP = 16;
// static const int MAX_SPEED = 10;



// using namespace webots;
// using namespace std;
// void turnArm(float angle);
// void slide(bool reverse=false, bool ball = false);

// Motor *wheel[4];
// Motor *hinge_mot;
// Motor *base1Mot;
// Motor *base2Mot;
// Motor *actMot;

// Robot *robot = new Robot();
// //Compass *compass;
// PositionSensor *psensor;
// PositionSensor *psensorArm;
// PositionSensor *basepos1;
// PositionSensor *basepos2;
// PositionSensor *actPos;


// // get the time step of the current world.
// int timeStep = (int)robot->getBasicTimeStep();

// int main(int argc, char **argv)
// {

  
//   for (int i = 0; i < 4; i++)
//   {
//     wheel[i] = robot->getMotor(wheel_names[i]);
//     wheel[i]->setPosition(INFINITY);

//     wheel[i]->setVelocity(0.0);
//   }

//   robot->step(100);
  
//   // initializing hinge motor
//   hinge_mot = robot->getMotor("Arm_mot");
//   hinge_mot->setPosition(INFINITY);
//   hinge_mot->setVelocity(0.0);
//   robot->step(100);
 
//   ///////////////////////////////////////////////////////////////////////////////
//   psensorArm = robot->getPositionSensor("Arm_pos");
//   psensorArm->enable(timeStep);
//   robot->step(100);
//   /////////////////////////////////////////////////////////////////////////////////////////////
//   basepos1 = robot->getPositionSensor("basepos1");
//   basepos1->enable(timeStep);
//   robot->step(100);
//   ////////////////////////////////////////////////////////////////////////
//   basepos2 = robot->getPositionSensor("basepos2");
//   basepos2->enable(timeStep);
//   robot->step(100);
//   //////////////////////////////////////////////////////////////////////
//   base1Mot = robot->getMotor("base1Mot");
//   base1Mot->setPosition(INFINITY);
//   base1Mot->setVelocity(0.0);
//   robot->step(100);

//   base2Mot = robot->getMotor("base2Mot");
//   base2Mot->setPosition(INFINITY);
//   base2Mot->setVelocity(0.0);
//   robot->step(100);

//   actPos = robot->getPositionSensor("actPos");
//   actPos->enable(timeStep);
//   robot->step(100);

//   actMot = robot->getMotor("actMot");
//   actMot -> setPosition(INFINITY);
//   actMot->setVelocity(0.0);
//   robot->step(100);


// while (robot->step(timeStep) != -1)
//   {
//     turnArm(5);
//     slide(false, true);
//     turnArm(-90);
//     turnArm(90);
//     slide(true,false);
//     turnArm(-90);
    
    
   
  
//     while (robot->step(timeStep) != -1){
//     }
//   }; 

//   delete robot;
//   return 0;


// int count = 0;
//   double pos1 = basepos1->getValue();
//   double pos2 = basepos2->getValue();
//   double prevoiusPos1;
//   double prevoiusPos2;

//   while (robot->step(timeStep) != -1)
//   {
    
//     prevoiusPos1 = pos1;
//     prevoiusPos2 = pos2;
//     pos1 = basepos1->getValue();
//     pos2 = basepos2->getValue();

//     if ( abs(prevoiusPos1 - pos1) < 0.001 && abs(prevoiusPos2-pos2) < 0.001 ) {
//       count += 1;
//     }
//     else{
//       count = 0;
//     }

//     if (count > 100) {
//       break;
//     }
//   }
//   cout << "Loop Broke" << endl;
 
// }


// void turnArm(float angle){

//   // int starting_position = psensor -> getValue();
//   float velocity = angle / abs(angle);
//   float startPos = psensorArm->getValue();
//   while (robot->step(timeStep) != -1)
//   {
//     float pos = psensorArm->getValue();
//     hinge_mot->setVelocity(velocity);
//     // cout << (abs(startPos - pos) / 3.142) * 180 << "      " << abs(angle) << endl;
//     if ((abs(startPos - pos) / 3.142) * 180 > abs(angle))
//     {
//       hinge_mot->setVelocity(0);
//       break;
//     }

    
//   }
  
// }

// void slide(bool reverse, bool ball)
// {

//   float END_POS;
//   if (ball) END_POS = 0.025;
//   else END_POS = 0.019;

//   base1Mot->setVelocity(0.04); 
//   base2Mot->setVelocity(0.04);

//   if(reverse){


// if(ball){
//       base1Mot -> setPosition(-0.02); 
//       base2Mot -> setPosition(0.02);
//     }else{
//       base1Mot -> setPosition(0.01); 
//       base2Mot -> setPosition(-0.01);
//     }

//   }else{
//     base1Mot -> setPosition(-END_POS); //done
//     base2Mot -> setPosition(END_POS);
//   }
  
//   int count = 0;
//   double pos1 = basepos1->getValue();
//   double pos2 = basepos2->getValue();
//   double prevoiusPos1;
//   double prevoiusPos2;

//   while (robot->step(timeStep) != -1 && !reverse)
//   {
//     prevoiusPos1 = pos1;
//     prevoiusPos2 = pos2;
//     pos1 = basepos1->getValue();
//     pos2 = basepos2->getValue();

//     if ( abs(prevoiusPos1 - pos1) < 0.001 && abs(prevoiusPos2-pos2) < 0.001 ) {
//       count += 1;
//     }
//     else{
//       count = 0;
//     }

//     if (count > 100) {
//       break;
//     }
//   }
// }