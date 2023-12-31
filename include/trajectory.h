#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "vex.h"
#include "timer.h"
#include "PID.h"

class Trajectory
{
public:
    Trajectory();
    void setJumpTime(double);
    void setAngTol(double);
    void setPIDR(float _p, float _i, float _d, float IMax, float _IRange);
    bool targetArrived();
    void update();

protected:
    PID pidr;
    Timer myTimer;
    bool arrived;
    double jumpTime;
    double maxSpeed;
    double angTol;
    double targetAng;
    double velocityR;
    void initPID();
    void pidControlR(PID &pid);
};

#endif