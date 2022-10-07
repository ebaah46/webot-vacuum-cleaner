// File:          create_avoid_obstacles.cpp
// Date:
// Description:   webots controller for irobot
// Author:        Emmanuel Baah
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
//Use webots namespace for all classes
using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // instance of a device of the robot.
    DistanceSensor *ps[8];
    char psNames[8][4] = {
            "ps0","ps1","ps2","ps3",
            "ps4","ps5","ps6","ps7"
    };
  // Enable the sensors
  for (int i=0;i<8;i++){
      ps[i] = robot->getDistanceSensor(psNames[i]);
      ps[i]->enable(TIME_STEP);
  }
  // Initialize motors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);


  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    double psValues[8];
    for (int i=0;i<8;i++)
        psValues[i] = ps[i]->getValue();

    // Process sensor data here.
    // Get obstacle on left
    bool leftObstacle = psValues[5] > 80.0 || psValues[6] > 80.0 || psValues[7] > 80.0;
    bool rightObstacle = psValues[0] > 80.0 || psValues[1] > 80.0 || psValues[2] > 80.0;

    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

    // Check for obstacles around
    if (leftObstacle){
        // turn right
        leftSpeed = 0.5 * MAX_SPEED;
        rightSpeed = -0.5 * MAX_SPEED;
    }
     else if (rightObstacle){
          // turn left
          leftSpeed = -0.5 * MAX_SPEED;
          rightSpeed = 0.5 * MAX_SPEED;
      }
    // Set actuator commands
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  //  cleanup code.

  delete robot;
  return 0;
}
