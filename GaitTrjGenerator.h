#ifndef GAIT_TRJ_GENERATOR_H
#define GAIT_TRJ_GENERATOR_H

#include <cmath>
#include "GeoDefinition.h"

namespace RobotHighLevelControl
{
    class ParamCXB;
}

namespace Model
{

// Hermit interpolation given the position and velocity at the beginning and the ending point
// Note: the time is normalized.
void HermitInterpolate(
        double T,
        double y0, 
        double v0, 
        double y1, 
        double v1, 
        double t, 
        double& y, 
        double& v)
{
    double v0r = v0 * T;
    double v1r = v1 * T;
    double a =  2 * (y0 - y1) + v0r + v1r;
    double b = -3 * (y0 - y1) - 2 * v0r - v1r;
    double c = v0r;
    double d = y0;

    y = a * t*t*t + b * t*t + c * t + d;
    v = (3*a*t*t   +2*b* t   + c) / T;
}

// Cubic spline only for 2 segment interpolation
// Note: the time is normalized
void Spline2SegInterpolate(
        double T,
        double y0, 
        double v0, 
        double y2, 
        double v2, 
        double y1,
        double t1, // t1 is normalized 
        double t, 
        double& y, 
        double& v)
{
    double t0 = 0;
    double t2 = T;
    double h0 = t1*T - t0;
    double h1 = t2 - t1*T;

    double lambda1 = h1/(h0+h1);
    double mu1 = h0/(h1+h0);
    double g1 = 3*(mu1*(y2-y1)/h1 + lambda1*(y1-y0)/h0);

    double v1 = (g1 - lambda1*v0 - mu1*v2)/2;
    double tr = 0;

    // now we have the velocity at (t1, y1) point, use HermitInterpolate to 
    // finish the next thing
    if ( t < t1 )
    {
       tr = (t*T - t0) / h0;
       HermitInterpolate(T*t1, y0, v0, y1, v1, tr, y, v); 
    } 
    else 
    {
       tr = (t - t1) * T / h1;
       HermitInterpolate(T*(1 - t1), y1, v1, y2, v2, tr, y, v); 
    }
}

}

#endif
