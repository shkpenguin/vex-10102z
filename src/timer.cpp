#include "timer.h"
#include "vex.h"
#include "robot-config.h"

Timer::Timer()
{
    startTime = Brain.Timer.value();
}

void Timer::reset()
{
    startTime = Brain.Timer.value();
}

int Timer::getTime() const 
{
    return floor((Brain.Timer.value() - startTime) * 1000);
}

double Timer::getTimeDouble() const
{
    return Brain.Timer.value() - startTime;
}

