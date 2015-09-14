#include <iostream>
#include "GaitTrjGenerator.h"

using namespace Model;
using namespace std;

int main(int argc, char *argv[])
{
    double timeNow;
    unsigned int totalPeriodCount = 20;
    double stepLength = 400;
    double Lside = 0;
    double rotationAngle = 0;
    double duty = 0.55;
    double stepHeight = 60; //positive value
    double T = 1.0;
    double standHeight = 700;
    double legTipPoints[18];
    double tdDeltaMidLeg = 5;
    double tdDeltaSideLeg = 5;

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
        for (int i = 0; i < 18; ++i) {
            cout << legTipPoints[i] << "   ";
        }
        cout << endl;

    }

    return 0;
}
