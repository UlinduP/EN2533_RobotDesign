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
#define MAX_SPEED_line_follow 7
#define TIME_STEP 16
#define WHEEL_RADIUS 0.025

using namespace webots;
using namespace std;

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

double kingAngle[2];

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

void moveForward(float distance);
double moveForwardDist() ;
float getBearing(double x, double z);
void turnRobot(float angle) ;
void kingPos();
int checkPath(double v_angle, double j_angle, double r_angle);
void placeRook();
void chessBoard();

int main(int argc, char **argv){
  
  //cout<<5<<endl;
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

    king_color_detector=robot->getDistanceSensor("king_color_detector");
    king_color_detector->enable(TIME_STEP);

    rotational_motor = robot->getMotor("rotational_motor");
    rotational_sensor = robot->getPositionSensor("rotational_sensor");
    rotational_sensor->enable(TIME_STEP);

    linear_motor = robot->getMotor("linear_motor");
    linear_motor->setPosition(0.0);

    laser_sensor = robot->getDistanceSensor("laser_sensor");
    laser_sensor->enable(TIME_STEP);

    // Initializing the compass
    compass = robot->getCompass("compass");
    compass->enable(TIME_STEP);

    // initializing position sensors
    right_front_wheel_pos_sensor = robot->getPositionSensor("right_front_wheel_pos_sensor");
    right_front_wheel_pos_sensor->enable(TIME_STEP);
  

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while (robot->step(TIME_STEP) != -1)
    {

      kingPos();
      //cout<<kingAngle[0]<<endl;
      //turnRobot(0);

      chessBoard();

      delete robot;
      return 0;

    }
}


void moveForward(float distance)  // 0.75 in scale is 15 in sensor val
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
      if(abs(right_front_wheel_pos_sensor -> getValue() - startPos) > distance)
      {
        motors[0]->setVelocity(0);
        motors[1]->setVelocity(0);
        motors[2]->setVelocity(0);
        motors[3]->setVelocity(0);
        break;
      }
    }
  }

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
