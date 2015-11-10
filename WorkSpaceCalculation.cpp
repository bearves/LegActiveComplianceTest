#include "LegKinematicsIV.h"
#include <iostream>
#include <cmath>

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    Leg a;

    double len, ang;
    double tipPositionPole[3];
    double jointLength[3];

    a.SetID(Leg::LEG_ID_MB);

    for(int i = 0; i < 1000; i++)
    {
        for(int j = 0; j < 1000; j++)
        {
            len = 0.5 + i * 0.0005;
            ang = -PI / 6.0 + j * (PI / 3.0) / 1000.0;

            tipPositionPole[2] = len;
            tipPositionPole[0] = ang;
            tipPositionPole[1] = 0;

            a.InverseSolutionPole(tipPositionPole, jointLength, false);

            bool inSpace = true;
            for(int k = 0; k < 3; k++)
            {
                if (!isfinite(jointLength[k]) || jointLength[k] < 0 || jointLength[k] > 0.16)
                    inSpace = false;
            }
            cout << len << "   " << ang << "   " << (inSpace ? 1 : 0) << endl;
        }
    }
    
    

    return 0;
}
