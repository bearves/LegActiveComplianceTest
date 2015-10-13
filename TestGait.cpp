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
    Model::HopTrjGenerator hopGenerator;

    hopGenerator.Initialize();

    leg.SetID(0);

    for(int timeCounter = 0; timeCounter < 6000; timeCounter++)
    {
        timeNow = timeCounter / 1000.0;
        hopGenerator.HopOnce
            ( timeNow,
              false,
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
