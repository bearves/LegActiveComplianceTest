#include <iostream>
#include "GaitTrjGenerator.h"
#include "Planners.h"
#include "LegKinematicsIV.h"

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    double timeNow;
    double legTipPoints[3] = {0.0, -0.00, -0.645};
    double legTipPoints2[3] = {-0.0, 0, -0.64};
    double screwLength[3];
    double screwLength2[3];
    Model::Leg leg;

    leg.SetID(0);

    leg.InverseSolution(legTipPoints, screwLength, false);

    for( auto a : screwLength)
        cout << a << "\n";

    cout << "\n";

    leg.InverseSolution(legTipPoints2, screwLength2, false);

    for( auto a : screwLength2)
        cout << a << "\n";

    cout << "\n";

    for( int i = 0; i < 3; i++)
        cout << 65536*1.5*(screwLength[i] - screwLength2[i])/0.005 << "\n";
    cout << endl;

/*

    double Tforward = 0.4;
    double tdAngle = 0.2;
    double tdAngleVel = -0.8;

    for (int i = 0; i < Tforward*1000; i++)
    {
        double angRef, velRef;
        double tf = i / (Tforward * 1000);
        Model::Spline2SegInterpolate(
                Tforward,
                -0.3 + tdAngleVel*0.15, tdAngleVel,
                tdAngle, tdAngleVel,
                tdAngle, 0.75, // t1 is normalized 
                tf, 
                angRef, velRef);
        cout << tf*Tforward << " " << angRef << " " << velRef;
        Model::HermitInterpolate(
                Tforward,
                -0.3 + tdAngleVel*0.15, tdAngleVel,
                tdAngle, tdAngleVel,
                tf,
                angRef, velRef);
        cout << "  "  << angRef << " " << velRef << endl;
    }
    */
    return 0;
}
