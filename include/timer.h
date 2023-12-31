#ifndef TIMER_H_
#define TIMER_H_

class Timer
{
private:
    double startTime;
public:
    Timer();
    Timer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;

};

#endif