#include "trajectory.h"
#include "utils.h"
#include "calc.h"

using namespace vex;


void Trajectory::initPID() {
    pidr.setErrorTolerance(3);
    pidr.setCoefficient(0.88, 3.22, 0.16); 
    pidr.setDTolerance(50);
    pidr.setIMax(60);
    pidr.setIRange(5);
    pidr.setJumpTime(200);
}
void Trajectory::setJumpTime(double t) {
    jumpTime = t;
}
void Trajectory::setAngTol(double tol) {
    angTol = tol;
}
void Trajectory::setPIDR(float _p, float _i, float _d, float _IMax, float _IRange) {
    pidr.setCoefficient(_p, _i, _d);
    pidr.setIMax(_IMax);
    pidr.setIRange(_IRange);
}
bool Trajectory::targetArrived() {
    return arrived;
}
void Trajectory::update() {

}

void Trajectory::pidControlR(PID &pid) {
    double ang = IMUHeading();
    double da = targetAng - ang;
    while (da > 180) {
        da -= 360;
        ang += 360;
        pid.setTarget(targetAng);
    }
    while (da < -180) {
        da += 360;
        ang -= 360;
        pid.setTarget(targetAng);
    }
    pid.update(ang);
    velocityR = pid.getOutput();
    if (fabs(velocityR) > maxSpeed) { velocityR = sign(velocityR) * maxSpeed; }
    if (fabs(velocityR) < 1) { velocityR = 0; }
}





