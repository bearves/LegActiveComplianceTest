#include <iostream>
#include "GaitTrjGenerator.h"
#include "Planners.h"
#include "LegKinematicsIV.h"

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    double timeNow;
    unsigned int totalPeriodCount = 20;
    double stepLength = 0;
    double Lside = 0;
    double rotationAngle = 0;
    double duty = 0.51;
    double stepHeight = 80; //positive value
    double T = 3;
    double standHeight = 710;
    double legTipPoints[18];
    double screwLength[3];
    double tdDeltaMidLeg = 0;
    double tdDeltaSideLeg = 0;
    Model::Leg leg;

    leg.SetID(0);

    for(int timeCounter = 0; timeCounter < 20000; timeCounter++)
    {
        timeNow = timeCounter / 1000.0;
        walk_cxb(
                timeNow,
                totalPeriodCount,
                stepLength,
                Lside,
                rotationAngle,
                duty,
                stepHeight,
                T,
                standHeight,
                tdDeltaMidLeg,
                tdDeltaSideLeg,
                legTipPoints);

        cout << timeNow << "  "; 
        for (int i = 0; i < 6; ++i) {
            double *positionPole = &legTipPoints[i*3];
            double l = positionPole[2]; 
            double ty = positionPole[0];
            double tx = positionPole[1];
            double positionCart[3];
            positionCart[0] = l * sin(ty);// X direction
            positionCart[1] = l * sin(tx) * cos(ty); // Y direction
            positionCart[2] = -l * cos(tx) * cos(ty); // Z direction
            for(int j = 0; j < 3; j++)
            {
                cout << positionCart[j] << "   ";
            }
        }

        leg.InverseSolutionPole(legTipPoints, screwLength, false);

        for(int j = 0; j < 3; j++)
        {
            cout << screwLength[j] << "   ";
        }
        cout << endl;
    }

    return 0;
}
