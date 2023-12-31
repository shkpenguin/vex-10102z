#include "usercontrol.h"
#include "manul-function.h"
#include "robot-config.h"
#include "vex.h"
#include "utils.h"
#include "manul-function.h"
#include "timer.h"
#include "controller.h"
#include "iostream"

using namespace vex;

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

    controlIntake();
    controlCatapult();
    controlWings();
    //controlArm();
    defineDriveMode();
    controlHang();
    // Brain.Screen.print("Inertial: %.2f,     IMU: %.2f \n", Inertial.rotation(rotationUnits::deg), IMUHeading());
    std::cout << Inertial.rotation(rotationUnits::deg)  <<  IMUHeading()<< std::endl;

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}