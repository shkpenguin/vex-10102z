#include "auto-function.h"
#include "trajectory.h"
#include "robot-config.h"
#include "vex.h"
#include "utils.h"
#include "calc.h"
#include "geometry.h"
#include "position.h"
#include "chassis.h"

using namespace vex;

void quickTurnTo(double _tarAng, double _maxSpeed)
{
    while (_tarAng >= 360) { _tarAng -= 360; }
    while (_tarAng < 0) { _tarAng += 360; }

    auto pidr = PID();
    if ( fabs(_tarAng - IMUHeading()) < 10 || fabs(fabs(_tarAng - IMUHeading()) -360) < 10 ) {
        pidr.setCoefficient(2, 0.0, 0.05); // for small angle
    } 
    else if ( fabs(_tarAng - IMUHeading()) < 50 || fabs(fabs(_tarAng - IMUHeading()) -360) < 50 ) {
        pidr.setCoefficient(1, 0.0, 0.05); 
    }
    else {
        pidr.setCoefficient(0.75, 0.0, 0.05); // for larger angle
    }    
    //pidr.setTarget(_tarAng);
    pidr.setArrived(false);
    pidr.setDTolerance(30);
    pidr.setFirstTime();
    pidr.setIMax(40);
    pidr.setIRange(5);
    pidr.setJumpTime(200); 
    pidr.setErrorTolerance(3);
    
    Timer myTimer;
    while (!pidr.targetArrived() && myTimer.getTime() <= 800)
    {
        double ang = IMUHeading();
        double d = _tarAng - ang;
        while (d > 180) {
            d -= 360;
            _tarAng -= 360;
        }
        while (d < -180) {
            d += 360;
            _tarAng += 360;
        }
        pidr.setTarget(_tarAng);
        pidr.update(ang);
        double velocityR = pidr.getOutput();
        if (abs(velocityR) > _maxSpeed) velocityR = sign(velocityR) * _maxSpeed;
        if (abs(velocityR) < 1) velocityR = 0;
        if (abs(velocityR) > 100) { velocityR = 100 * sign(velocityR); }
        LeftMotor.spin(vex::directionType::fwd, velocityR*127, voltageUnits::mV);
        RightMotor.spin(vex::directionType::rev, velocityR*127, voltageUnits::mV);     
        this_thread::sleep_for(20);    
    }
    LeftMotor.stop(hold);
    RightMotor.stop(hold);
    
}


void quickMoveDistance(double _distance, double _maxSpeed, double _maxTime)
{

    auto pidv = PID();
    if ( _distance < 30 ) 
    {
        pidv.setCoefficient(5.0, 0.0, 0.0);
    } 
    else if ( _distance < 90 )
    {
        pidv.setCoefficient(4.0, 0.0, 0.0);
    } 
    else
    {
        pidv.setCoefficient(3.0, 0.0, 0.0);
    }
    
    pidv.setArrived(false);
    pidv.setDTolerance(30);
    pidv.setFirstTime();
    pidv.setIMax(20);
    pidv.setIRange(10);
    pidv.setJumpTime(100); 
    pidv.setErrorTolerance(1);
    
    double time = 1000 * _distance / 100;
    if (time <1300) time = 1300;
    time = fmin(time, _maxTime);

    LeftMotor.setPosition(0.0, rev);
    RightMotor.setPosition(0.0, rev);
   
    Timer myTimer;
    while (!pidv.targetArrived() && myTimer.getTime() <= time) 
    {
        double revolution = (LeftMotor.position(rev) + RightMotor.position(rev))/2 ;
        double accumulatedLength = M_PI * Diameter * revolution * GearRatio;
        double d = _distance - accumulatedLength;

        pidv.setTarget(_distance);
        pidv.update(accumulatedLength);
        double velocityV = pidv.getOutput();
        if (abs(velocityV) > _maxSpeed) velocityV = sign(velocityV) * _maxSpeed;
        if (abs(velocityV) < 1) velocityV = 0;
        if (abs(velocityV) > 100) { velocityV = 100 * sign(velocityV); }
        LeftMotor.spin(vex::directionType::fwd, velocityV*127, voltageUnits::mV);
        RightMotor.spin(vex::directionType::fwd, velocityV*127, voltageUnits::mV);   
        this_thread::sleep_for(20);    
    }
    LeftMotor.stop(hold);
    RightMotor.stop(hold);
}

void quickMoveToWithHeading(Point _tarPos, double _tarAng, double _maxSpeed)
{
    Point initPos = Position::getInstance()->getPos();

    auto pidv = PosPID();
    pidv.setCoefficient(4, 0, 0); 
    pidv.setArrived(false);
    pidv.setDTolerance(30);
    pidv.setFirstTime();
    pidv.setIMax(20);
    pidv.setIRange(10);
    pidv.setJumpTime(100); 
    pidv.setErrorTolerance(3);
    pidv.setTarget(_tarPos);

    auto pidr = PID();
    pidr.setCoefficient(0.7, 0.0, 0.05); 
    //pidr.setTarget(_tarAng);
    pidr.setArrived(false);
    pidr.setDTolerance(30);
    pidr.setFirstTime();
    pidr.setIMax(40);
    pidr.setIRange(5);
    pidr.setJumpTime(200); 
    pidr.setErrorTolerance(3);
    pidr.setTarget(_tarAng);

    auto pidt = PID();
    pidt.setTarget(0);
    pidt.setCoefficient(2, 0, 1);
    pidt.setJumpTime(200);
    pidt.setIMax(7);
    pidt.setIRange(10);
    pidt.setTarget(0);


    double dis = (_tarPos - initPos).mod();
    double time = 1000 * dis / 100;
    if (time < 1300) time = 1300;

    Vector velocity;
    Timer myTimer;
    while (!pidv.targetArrived() && !pidr.targetArrived() && myTimer.getTime() <= time) 
    {
        double x1 = initPos._x, y1 = initPos._y;    
        double x2 = _tarPos._x, y2 = _tarPos._y;
        Point curPos = Position::getInstance()->getPos();
        double x3 = curPos._x, y3 = curPos._y;
        double A = y1 - y2;
        double B = x2 - x1;
        double C = x1 * y2 - x2 * y1;
        double M = (A != 0 || B != 0) ? abs(A * x3 + B * y3 + C) / sqrt(A * A + B * B) : 0;
        Vector fixVel = Vector(((x3*(x1 - x2) + y3*(y1 - y2))*(x1 - x2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - ((y2*(x1 - x2) - x2*(y1 - y2))*(y1 - y2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - x3,
                            ((y2*(x1 - x2) - x2*(y1 - y2))*(x1 - x2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) + ((x3*(x1 - x2) + y3*(y1 - y2))*(y1 - y2))/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) - y3);
        pidt.update(-M);
        if (fixVel.mod() != 0){
            fixVel = fixVel / fixVel.mod();
            double output = pidt.getOutput();
            if (output >= 20) output = 20;
            fixVel = fixVel * output;
        }
        pidv.update(curPos);
        double speed = pidv.getOutput();
        if (speed > 100)  speed = 100;
        Vector d = (_tarPos - curPos);
        if (d.mod() != 0)
            d = d / d.mod();
        velocity = speed * d;
        if (velocity.mod() > _maxSpeed) velocity = velocity / velocity.mod() * _maxSpeed;

        double ang = IMUHeading();
        double da = (_tarAng - ang);
        while (da > 180){
            da -= 360;
            ang += 360;
        }
        while (da < -180){
            da += 360;
            ang -= 360;
        }
        pidr.update(ang);
        double velocityR = pidr.getOutput();
        if (abs(velocityR) > _maxSpeed) velocityR = sign(velocityR) * _maxSpeed;
        if (abs(velocityR) < 1) velocityR = 0;
        Chassis::getInstance()->autoSetWorldVel(velocity, velocityR);
        this_thread::sleep_for(50);
   
    }
    LeftMotor.stop(hold);
    RightMotor.stop(hold);
    
}