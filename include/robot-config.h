#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

#include "driveMode.h"
#include "vex.h"

using namespace vex;

extern competition Competition;
extern brain Brain;
extern motor leftFront;
extern motor leftBack;
extern motor leftMiddle;
extern motor rightFront;
extern motor rightBack;
extern motor rightMiddle;
extern motor catapult;
extern motor arm;
extern motor intake;
extern digital_out wings;
extern digital_out hang;
extern inertial Inertial;
extern controller Controller1;
extern  motor_group LeftMotor;
extern motor_group RightMotor;
extern DriveMode driveMode;
extern int joystickDeadZone;

const double IMUCoefficient = 1;
const double GearRatio = 0.5;
const double Diameter = 4;
const int motorVoltage = 127; // unit: 0.01 V
const double RefreshTime = 10; 
const double positionRefreshTime = 10;
const double wheel_circumference = Diameter * M_PI;

#endif