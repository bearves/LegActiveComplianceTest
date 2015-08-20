/*
 * Gait.h
 *
 *  Created on: Nov 28, 2014
 *      Author: hex
 */
#ifndef GAIT_H_
#define GAIT_H_
#include "Aris_Control.h"
#include "Aris_ControlData.h"
#include "OnlinePlanner.h"

#include <string>
using namespace std;

#define GAIT_WIDTH 18
#define GAIT_HOME2START_LEN 4001
#define MOTOR_NUM 18

namespace RobotHighLevelControl
{

const int MapAbsToPhy[18]=
{
    10, 11, 9,
    12, 14, 13,
    17, 15, 16,
    6,  8,  7,
    3,  5,  4,
    0,  2,  1
};
const int MapPhyToAbs[18]=
{
    15, 17, 16,
    12, 14, 13,
    9,  11, 10,
    2,  0,  1,
    3,  5,  4,
    7,  8,  6
};


enum EGaitState
{
    NONE,
    GAIT_START,
    GAIT_RUN,
    GAIT_STOP,
};

enum EGAIT
{
    GAIT_NULL           = 0,
    GAIT_STANDSTILL     = 1,
    GAIT_HOME2START     = 2,
    GAIT_HOME           = 11,
    GAIT_ONLINE         = 12
};


class CGait
{
    public:
        CGait();
        ~CGait();
        //read txt to array
        static int InitGait(Aris::RT_CONTROL::CSysInitParameters& param);
        static int RunGait(double timeNow, EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data );
        static bool IsGaitFinished();
        static bool IsHomeStarted[AXIS_NUMBER];
        static bool IsConsFinished[AXIS_NUMBER];
        static int Gait_iter[AXIS_NUMBER];
        static int Gait_iter_count[AXIS_NUMBER];
        static EGaitState m_gaitState[AXIS_NUMBER];
        static OnlinePlanner onlinePlanner;

    private:
        static EGAIT m_currentGait[AXIS_NUMBER];
        static double m_screwLength[AXIS_NUMBER];
        static int m_commandMotorCounts[AXIS_NUMBER];
        static long long int m_gaitStartTime[AXIS_NUMBER];
        static int m_gaitCurrentIndex[AXIS_NUMBER];
        static Aris::RT_CONTROL::CMotorData m_standStillData[AXIS_NUMBER];
        static Aris::RT_CONTROL::CMotorData m_commandDataMapped[AXIS_NUMBER];
        static Aris::RT_CONTROL::CMotorData m_feedbackDataMapped[AXIS_NUMBER];
        static void MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data );
        static void MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data );
        static void CalculateActualMotorCounts( double screwLength [], int motorCounts []);
};

}

#endif /* GAIT_H_ */
