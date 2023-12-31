#include "utils.h"
#include "robot-config.h"
#include "vex.h"

using namespace vex;

double IMUHeading() {
    double heading = Inertial.rotation(rotationUnits::deg);
    // IMUCoefficient need be tested
    heading = heading * IMUCoefficient; // 
    while (heading < 0) heading += 360;
    while (heading > 360) heading -= 360;
    return heading;
}

void setIMUHeading(double degree)
{
    Inertial.setRotation(degree, rotationUnits::deg);
}

void openWings(){
  wings.set(true);
}

void closeWings(){
  wings.set(false);
}

void intakeTriball(float percent) {
  intake.spin(forward, motorVoltage * percent, vex::voltageUnits::mV);
}

void stopIntake(){
  intake.stop();
}


void drive(float lpercent, float rpercent) {
    RightMotor.spin(forward, motorVoltage * rpercent, vex::voltageUnits::mV);
    LeftMotor.spin(forward, motorVoltage * lpercent, vex::voltageUnits::mV);
 
}









