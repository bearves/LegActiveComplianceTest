#ifndef ONLINE_PLANNER_H
#define ONLINE_PLANNER_H

#include <cmath>
#include "Aris_ControlData.h"

// The planner to generate the joint trajectory for push recovery
class OnlinePlanner
{
public:
    enum ONLINE_GAIT_STATE
    {
        OGS_ONLINE_WALK    = 1,
        OGS_ONLINE_GOTO_START_POINT = 2,
        OGS_RETURNING      = 3,
        OGS_OFFLINE        = 5
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

    // Generate the joint trajectory due to the external force, should be called in each cycle.
    // TODO: the seconde param should be replaced as the model input, not the actual input
    int GenerateJointTrajectory(double timeNow, Aris::RT_CONTROL::CMachineData& machineData, double jointLengthList[]);
    // Get the joint length of initial position, used for go to initial position before starting
    int GetInitialJointLength(double jointLengthList[]);
    // Get the forward solution of the robot
    int GetForwardLegPositions(double jointLengthList[], double legTipPositions[]);

    ONLINE_GAIT_STATE GetCurrentState() const { return olgaitState; };

private:
    static const double BASIC_FEET_POSITION[18];
    static const double BASIC_BODY_POSITION[6];


    ONLINE_GAIT_STATE olgaitState;

    double initialFeetPosition[18];
    double initialBodyPosition[6];
    double legGroupPosition[6];
    double legGroupPositionDot[6];
    double feetPosition[18];
    double bodyPosition[18];

    // Assign the position of leg group from the virtual model to the position of acutal legs
    int CalculateEachLegPosition();
}; 
#endif
