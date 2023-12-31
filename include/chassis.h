#ifndef CHASIS_H_
#define CHASIS_H_

#include "geometry.h"
#include "vex.h"

class Chassis
{
private:

    //机器人坐标系速度，机器朝向为y轴，机器人朝向的右边为x轴
    Vector robotVel;

    //机器人角速度
    double robotVelR;

    //机器人坐标系下手动控制的速度
    Vector manualVel;
    double manualVelR;
    //机器人坐标系下自动控制的速度
    Vector autoVel;
    double autoVelR;

    double wheelVel[2] = {0};
    double wheelVolt[2] = {0};

    vex::brakeType stopBrakeType;


    Vector calcRobotVel(Vector Vel);
    void calcWheelVel();
    void calcWheelVolt();
    void setMotorVolt();

    // double updateRobotHeading();

public:
    static Chassis *getInstance(){
        static Chassis *c = NULL;
        if (c == NULL){
            c = new Chassis();
        }
        return c;
    }
    static void deleteInstance(){
        Chassis *c = Chassis::getInstance();
        if(c != NULL){
            delete c;
            c = NULL;
        }
    }
    Chassis();
    void setStopBrakeType(vex::brakeType brake);
    void manualSetWorldVel(Vector Vel, double VelR);
    void manualSetRobotVel(Vector Vel, double VelR);
    void autoSetWorldVel(Vector Vel, double VelR);
    void autoSetRobotVel(Vector Vel, double VelR);
    void chassisBrake(vex::brakeType brake);
    void chassisRun();
};

void updateChassis();

#endif