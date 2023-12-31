#include "robot-config.h"
#include "driveMode.h"
#include "vex.h"
using namespace vex;

competition Competition;
brain Brain;
motor leftFront = motor(PORT20, ratio6_1, true);
motor leftBack = motor(PORT10, ratio6_1, true);
motor leftMiddle = motor(PORT19, ratio6_1, false);
motor rightFront = motor(PORT11, ratio6_1, false);
motor rightBack = motor(PORT1, ratio6_1, false);
motor rightMiddle = motor(PORT12, ratio6_1, true);
motor catapult = motor(PORT9, ratio36_1, false);
//motor arm = motor(PORT11, ratio36_1, true);
motor intake = motor(PORT15, ratio6_1, true);
digital_out wings = digital_out(Brain.ThreeWirePort.G);
digital_out hang = digital_out(Brain.ThreeWirePort.H);
inertial Inertial = inertial(PORT2);
controller Controller1 = controller(primary);
motor_group LeftMotor = motor_group(leftFront, leftMiddle, leftBack);
motor_group RightMotor = motor_group(rightFront, rightMiddle, rightBack);

DriveMode driveMode = DriveMode::Tank; // Arcade or Tank
int joystickDeadZone = 5;
