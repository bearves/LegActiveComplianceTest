#include <iostream>
#include "GaitTrjGenerator.h"
#include "Planners.h"
#include "LegKinematicsIV.h"

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    double timeNow;
    double legTipPoints[3] = {-0.015, -0.0, -0.66};
    double legTipPoints2[3] = {-0.0, 0, -0.66};
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

    return 0;
}
