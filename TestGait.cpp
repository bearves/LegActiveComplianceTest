#include <iostream>
#include "GaitTrjGenerator.h"
#include "Planners.h"
#include "LegKinematicsIV.h"

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    double timeNow;
    double legTipPoints[18];
    double screwLength[3];
    Model::Leg leg;

    leg.SetID(0);

    double TotalT = 4;
    double tr = 0;
    double p,v;

    for(int timeCounter = 0; timeCounter < 4000; timeCounter++)
    {
        timeNow = timeCounter / 1000.0;

        cout << timeNow << "  "; 
        tr = timeNow/TotalT; 

        Spline2SegInterpolate(
                TotalT,
                -150, -60,
                200, 0,
                200, 
                0.5, tr, p, v);

        cout << p << "   " << v << "   " << endl;
    }

    return 0;
}
