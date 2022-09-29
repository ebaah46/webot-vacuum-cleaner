// File:          create_avoid_obstacles.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
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

 //

  // Main loop:
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
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
