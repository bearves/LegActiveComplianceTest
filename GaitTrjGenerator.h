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

double WalkCxb(
        double timeFromStart,
        const RobotHighLevelControl::ParamCXB& param,
        double *legTipPositionPole);

double WalkCxb(
        double timeFromStart,
        unsigned int totalPeriodCount,
        double stepLength,
        double Lside,
        double rotationAngle,
        double duty,
        double stepHeight , //positive value
        double T,
        double standHeight,
        double tdDeltaMidLeg,
        double tdDeltaSideLeg,
        double *legTipPositionPole);

class HopTrjGenerator
{
public:
    enum HOP_STATE : int
    {
        HOLD    = 0,
        THRUST  = 1,
        RETRACT = 2,
        LANDING = 3
    };

    HOP_STATE m_currentState;

    double m_lastStateShiftTime;

    double m_holdingTime;
    double m_thrustingTime;
    double m_retractingTime;

    double m_holdLength;
    double m_thrustLength;
    double m_retractLength;

    HopTrjGenerator();

    int Initialize();

    double HopOnce(
        double timeFromStart,
        bool   retractTrigger,
        double *legTipPositionPole);
};
}

#endif
