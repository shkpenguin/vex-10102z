#include "position.h"
#include <cmath>
#include "timer.h"
#include "utils.h"
#include "robot-config.h"
#include <stdlib.h>

using namespace std;

Position::Position()
{
    resetMotorPosition();
    curIMUHeading = 0; // radius
    curLMileage = curRMileage = 0;
    lastLMileage = lastRMileage = 0;
    curLSpeed = curRSpeed = 0;
    lastLSpeed = lastRSpeed = 0;
    LSpeed[0] = LSpeed[1] = LSpeed[2] = 0;
    RSpeed[0] = RSpeed[1] = RSpeed[2] = 0;
    FiltLSpeed[0] = FiltLSpeed[1] = FiltLSpeed[2] = 0;
    FiltRSpeed[0] = FiltRSpeed[1] = FiltRSpeed[2] = 0;
    selfYSpeed = selfXSpeed = 0;
    globalYSpeed = globalXSpeed = 0;
    globalY = globalX = 0;
    lastTime = 0;
    sampleTime = 1;

    fliter_b[0] = 0.0675;
    fliter_b[1] = 0.1349;
    fliter_b[2] = 0.0675;

    fliter_a[0] = 1;
    fliter_a[1] = -1.143;
    fliter_a[2] = 0.4128;

}

/**
 * @brief return heading, 0 for front direction, range: 0 - 2M_PI 
 * 
 * @return double 
 */
void Position::updateInertialHeading()
{
    lastIMUHeading = curIMUHeading;
    curIMUHeading = deg2rad(IMUHeading());
}

void Position::updateAngleSpeed()
{
    // angleSpeed = (curIMUHeading - lastIMUHeading) * 1000 / sampleTime;
    // if (abs(angleSpeed) > 1000 || abs(angleSpeed) < 0.001) angleSpeed = 0;
}

/**
 * @brief 左定位轮走过距离 foot
 * 
 * @return float 
 */
void Position::updateLMileage()
{
    lastLMileage = curLMileage;

    curLMileage = LeftMotor.position(rev) * wheel_circumference * GearRatio;
}

/**
 * @brief 右定位轮走过距离 foot
 * 
 * @return float 
 */
void Position::updateRMileage()
{
    lastRMileage = curRMileage;

    curRMileage = RightMotor.position(rev) * wheel_circumference * GearRatio;
}

/**
 * @brief 左定位轮速度 foot/s
 * 
 * @return float 
 */
void Position::updateLSpeed()
{
    /*
    lastLSpeed = curLSpeed;

    double ret = (curLMileage - lastLMileage) * 1000 / sampleTime;                      //前向差分法
    if (abs(ret) > 1000 || abs(ret) < 0.001) ret = 0;
    
    // 滤波前的速度
    LSpeed[2] = LSpeed[1];
    LSpeed[1] = LSpeed[0];
    LSpeed[0] = ret;

    // 滤波后的速度
    FiltLSpeed[2] = FiltLSpeed[1];
    FiltLSpeed[1] = FiltLSpeed[0];
    FiltLSpeed[0] = (fliter_b[0] * LSpeed[0] + fliter_b[1] * LSpeed[1] + fliter_b[2] * LSpeed[2] - fliter_a[1] * FiltLSpeed[1] - fliter_a[2] * FiltLSpeed[2]) / fliter_a[0]; 
    if (abs(FiltLSpeed[0]) < 0.001) FiltLSpeed[0] = 0;

    curLSpeed = FiltLSpeed[0]; 
    // curLSpeed = ret;

    #ifdef debug
    if (CollectFlag  && isL) 
    {
        cout << ret <<" "<< FiltLSpeed[0] <<" "<< sampleTime <<"\n";
    }
    #endif
    */
}

/**
 * @brief 右定位轮速度 foot/s
 * 
 * @return float 
 */
void Position::updateRSpeed()
{
    /*
    lastRSpeed = curRSpeed;

    double ret = (curRMileage - lastRMileage) * 1000 / sampleTime;                      //前向差分法
    if (abs(ret) > 1000 || abs(ret) < 0.001) ret = 0;                                   //速度过大或过小，认为是噪声

    // 滤波前的速度
    RSpeed[2] = RSpeed[1];
    RSpeed[1] = RSpeed[0];
    RSpeed[0] = ret;

    // 滤波后的速度
    FiltRSpeed[2] = FiltRSpeed[1];
    FiltRSpeed[1] = FiltRSpeed[0];
    FiltRSpeed[0] = (fliter_b[0] * RSpeed[0] + fliter_b[1] * RSpeed[1] + fliter_b[2] * RSpeed[2] - fliter_a[1] * FiltRSpeed[1] - fliter_a[2] * FiltRSpeed[2]) / fliter_a[0];
    if (abs(FiltRSpeed[0]) < 0.001) FiltRSpeed[0] = 0;

    curRSpeed = FiltRSpeed[0];
    // curRSpeed = ret;
    */

}

/**
 * @brief 自身坐标系y轴速度 cm/s
 * 
 * @return double 
 */
void Position::updateSelfYSpeed()
{
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dy = curLSpeed * -cos(REncoderAngle) - curRSpeed * cos(LEncoderAngle);
    // selfYSpeed = Dy / D;

}

/**
 * @brief 自身坐标系x轴速度 cm/s
 * 
 * @return double 
 */
void Position::updateSelfXSpeed()
{
    // double D = -cos(REncoderAngle) * sin(LEncoderAngle) - cos(LEncoderAngle) * sin(REncoderAngle);
    // double Dx = curRSpeed * sin(LEncoderAngle) - curLSpeed * sin(REncoderAngle);
    // selfXSpeed = Dx / D;
}

/**
 * @brief 世界坐标系y轴速度 cm/s
 * 
 * @return float 
 */
void Position::updateGlobalYSpeed()
{
    lastglobalYSpeed = globalYSpeed;
    globalYSpeed = selfYSpeed * cos(curIMUHeading) - selfXSpeed * sin(curIMUHeading);
    if (abs(globalYSpeed) < 0.01) globalYSpeed = 0;
    if (abs(globalYSpeed) > 250) globalYSpeed = lastglobalYSpeed;
}

/**
 * @brief 世界坐标系x轴速度 cm/s
 * 
 * @return float 
 */
void Position::updateGlobalXSpeed()
{
    lastglobalXSpeed = globalXSpeed;
    globalXSpeed = selfYSpeed * sin(curIMUHeading) + selfXSpeed * cos(curIMUHeading);
    if (abs(globalXSpeed) < 0.01) globalXSpeed = 0;
    if (abs(globalXSpeed) > 250) globalXSpeed = lastglobalXSpeed;

    // # ifdef debug
    //     if (CollectFlag)
    //     {
    //         cout <<setiosflags(ios::fixed)<<setprecision(2)<< globalXSpeed << " " << globalYSpeed << "\n";
    //     }
    // # endif
}

/**
 * @brief 更新世界坐标系机器y轴位置（积分器）
 * 
 */
void Position::updateGlobalY()
{
    double change_in_distance = ( (curLMileage - lastLMileage) + (curRMileage - lastRMileage)) / 2;
    double d = change_in_distance * std::sin(- curIMUHeading);
    if (abs(d) < 0.001)
        return;
    else
        globalY = globalY + d;
}

/**
 * @brief 更新世界坐标系机器x轴位置（积分器）
 * 
 * @param speed 
 */
void Position::updateGlobalX()
{
    double change_in_distance = ( (curLMileage - lastLMileage) + (curRMileage - lastRMileage)) / 2;
    double d = change_in_distance * std::cos(curIMUHeading);     
    if (abs(d) < 0.001)
        return;
    else
        globalX = globalX + d;
}

/**
 * @brief 更新世界坐标系下机器位置（每tick调用）
 * 
 */
void Position::updatePos(){
    sampleTime = (Timer.getTimeDouble() - lastTime) * 1000;
    lastTime = Timer.getTimeDouble();

    updateInertialHeading();
    updateAngleSpeed();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfYSpeed();
    updateSelfXSpeed();
    updateGlobalYSpeed();
    updateGlobalXSpeed();
    updateGlobalY();
    updateGlobalX();
}

/**
 * @brief 获取当前位置
 * 
 * @return Point 
 */
Point Position::getPos() const{
    return Point(globalX, globalY);
}

double Position::getXSpeed() const{
    return globalXSpeed;
}

double Position::getYSpeed() const{
    return globalYSpeed;
}

double Position::getLMileage() const{
    return curLMileage;
}

double Position::getRMileage() const{
    return curRMileage;
}

void Position::resetXPosition(){
    globalX = 0;
}

void Position::resetYPosition(){
    globalY = 0;
}

void Position::setGlobalPosition(double _x, double _y){
    globalX = _x;
    globalY = _y;
}

void updatePosition()
{
    while(true)
    {
        Position::getInstance()->updatePos();
        this_thread::sleep_for(positionRefreshTime);
    }
    
}
