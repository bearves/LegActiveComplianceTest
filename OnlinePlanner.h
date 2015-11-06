#ifndef ONLINE_PLANNER_H
#define ONLINE_PLANNER_H

#include <cmath>
//#include <rtdk.h>
#include "Aris_ControlData.h"
#include "Planners.h"
#include "Aris_Message.h"

namespace RobotHighLevelControl
{

// The planner to generate the joint trajectory for push recovery
class OnlinePlanner
{
public:
    enum ONLINE_GAIT_STATE
    {
        OGS_ONLINE_WALK             = 1,
        OGS_ONLINE_GOTO_START_POINT = 2,
        OGS_OFFLINE                 = 5
    };
    OnlinePlanner(void);
    ~OnlinePlanner(void);

    // Load the trj data of sub planners
    int LoadData();
    // Intialize the planner, should be called when receiving the GAIT_INIT message
    // gaitMod tells the planner which task will be execuated
    int Initialize(int gaitMod);
    // Start the planner
    int Start(double timeNow);
    // Stop the planner, it cannot be start again
    int Stop(double timeNow);
    // Set offline to stop the online programming process
    int Offline();

    // Set gait parameter with the data
    int SetGaitParameter(const void* paramData, int dataLength, int gaitMod);

    // Generate the joint trajectory due to the external force, should be called in each cycle.
    int GenerateJointTrajectory(
            double timeNow, 
            double* jointStateInput, 
            Aris::RT_CONTROL::CForceData* forceData, 
            Aris::RT_CONTROL::CIMUData& imuFdbk,
            double* jointStateOutput,
            char* controlDataForLog);
    // Get the joint length of initial position, used for go to initial position before starting
    int GetInitialJointLength(double jointLengthList[]);

    ONLINE_GAIT_STATE GetCurrentState() const { return olgaitState; };

private:
    ONLINE_GAIT_STATE olgaitState;

    double initialFeetPosition[18];
    bool m_initialFlag;
    bool m_startOnlineGaitFlag;
    double m_destPoint[18]; // for PTP planner using

    GoToPointPlanner m_gotoPointPlanner;
    ImpedancePlanner m_impedancePlanner;
    Model::Leg legList[6];

    // Assign the position of leg group from the virtual model to the position of acutal legs
    int CalculateEachLegPosition();
}; 

}
#endif
