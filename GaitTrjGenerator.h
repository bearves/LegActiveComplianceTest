#ifndef GAIT_TRJ_GENERATOR_H
#define GAIT_TRJ_GENERATOR_H

#include <cmath>
#include "GeoDefinition.h"

namespace Model
{

double walk_cxb(
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
}

#endif
