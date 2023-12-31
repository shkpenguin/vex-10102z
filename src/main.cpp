/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       user                                                      */
/*    Created:      11/8/2023, 1:30:26 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "usercontrol.h"
#include "robot-config.h"
#include "autonomous.h"
#include "calc.h"
#include "controller.h"
#include "chassis.h"
#include "position.h"
#include "utils.h"
using namespace vex;


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  Inertial.calibrate();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  // Run the pre-autonomous function.
  //Competition.bStopAllTasksBetweenModes = true; 
  thread TController(defineController);
  /*
  thread UpdatePos(updatePosition);
  thread UpdateChassis(updateChassis);
  //need be setted
  Position::getInstance()->setGlobalPosition(0, 0);
  setIMUHeading(0);
  */
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
