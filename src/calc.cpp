#include "calc.h"
#include "math.h"

int sign(float x)
{
    return (x > 0) - (x < 0);
}

int sign(double x) 
{
    return (x > 0) - (x < 0);
}
double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

double clampScale(double value, double minValue, double maxValue){
    double result = 0;
    if (fabs(value) <= minValue) {
        result = 0;
    } else if (fabs(value) >= maxValue) {
        result = maxValue * sign(value);
    }
    return result;
}