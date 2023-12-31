#ifndef UTILS_H_
#define UTILS_H_

#include "vex.h"

double IMUHeading();
void setIMUHeading(double);
void openWings();
void closeWings();
void intakeTriball(float);
void stopIntake();
void drive(float, float);
void autoCatapult(int);

#endif