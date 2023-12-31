#include "chassis.h"
#include "robot-config.h"
#include "vex.h"
#include "utils.h"

using namespace vex;
using namespace std;

Chassis::Chassis()
{
    robotVel.resetV();
    manualVel.resetV();
    autoVel.resetV();
    stopBrakeType = coast;
    robotVelR = 0;
    manualVelR = 0;
    autoVelR = 0;
}

/// @brief convert global velocity to robot velocity
Vector Chassis::calcRobotVel(Vector Vel)
{
    return Vel.rotateTrans(IMUHeading());
}

/// @brief convert robot velocity to wheel velocity
void Chassis::calcWheelVel()
{
    double tranSpeed[2] = {0}, rotaSpeed[2] = {0};
    double robotSpeed = robotVel.mod();
    double maxAbsVel = 0;
    double maxAbsWheelVel = 100;
    for (int i = 0; i < 2; i++)
    {
        tranSpeed[i] = robotVel * Vector(1, 0).rotateTrans(0);
        if (abs(tranSpeed[i]) > maxAbsVel) maxAbsVel = abs(tranSpeed[i]);  
    }
    rotaSpeed[0] = robotVelR;    // for leftmotor
    rotaSpeed[1] = - robotVelR;    // for rightmotor
    for (int i = 0; i < 2; i++)
    {
        tranSpeed[i] = (maxAbsVel == 0) ? 0 : tranSpeed[i] / maxAbsVel * robotSpeed; 
        wheelVel[i] = tranSpeed[i] + rotaSpeed[i];
        if (abs(wheelVel[i]) > maxAbsWheelVel) maxAbsWheelVel = abs(wheelVel[i]);
        // cout << "rota" << i <<": " <<rotaSpeed[i] << endl;
    }

    // normalize wheel velocity to the range（-100 ~ 100）
    for (int i = 0; i < 2; i++)
    {
        wheelVel[i] = wheelVel[i] / maxAbsWheelVel * 100;
        // cout << "wheel" << i << ": " << wheelVel[i] << endl;
    }
}

/// @brief 【换算函数】将轮速转换为对应电机电压
void Chassis::calcWheelVolt()
{
    for (int i = 0; i < 2; i++)
    {
        wheelVolt[i] = wheelVel[i] * motorVoltage;
        if (abs(wheelVolt[i]) < 1000)
            wheelVolt[i] = 0;
    }
}

void Chassis::setMotorVolt()
{
    if (wheelVolt[0] == 0)
    {
        LeftMotor.stop(stopBrakeType);
    }
    else
    {
        LeftMotor.spin(directionType::fwd, wheelVolt[0], voltageUnits::mV);
    }

    if (wheelVolt[1] == 0)
    {
        RightMotor.stop(stopBrakeType);
    }
    else
    {
        RightMotor.spin(directionType::fwd, wheelVolt[1], voltageUnits::mV);
    }
}

/// @brief 强制底盘刹车（注意会收到电机驱动线程的影响，在使用该函数时确保不与电机驱动线程冲突）
/// @param brake braketype（coast，brake，hold）
void Chassis::chassisBrake(brakeType brake)
{
    wheelVolt[0] = 0;
    wheelVolt[1] = 0;    
    LeftMotor.stop(brakeType::brake);
    RightMotor.stop(brakeType::brake);
}

/// @brief drive  chassis， will be updated every freashtime in a thread
void Chassis::chassisRun()
{ 
    setMotorVolt();
}

/// @brief 【manual control】drive chassis based on global velocity
/// @param Vel global velocity，2 norm of the velocity in the range 0-100
/// @param VelR robot angular velociry，normalized to range -100 - 100
void Chassis::manualSetWorldVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    manualVel = calcRobotVel(Vel);
    manualVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【手动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::manualSetRobotVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    manualVel = Vel;
    manualVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【自动控制】通过世界坐标系速度驱动底盘
/// @param Vel 世界坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetWorldVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = calcRobotVel(Vel);
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【自动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetRobotVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = Vel;
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}


/**
 * @brief 设置停止时的刹车类型
 * 
 * @param brake 
 */
void Chassis::setStopBrakeType(brakeType brake){
    stopBrakeType = brake;
}

void updateChassis()
{
    while(true)
    {
        Chassis::getInstance()->chassisRun();
        this_thread::sleep_for(RefreshTime);
    }   
}

