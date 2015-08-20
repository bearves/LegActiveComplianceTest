#include "OnlinePlanner.h"

const double OnlinePlanner::BASIC_BODY_POSITION[] = {0, 0, 0, 0, 0, 0};
const double OnlinePlanner::BASIC_FEET_POSITION[] =
{
    -0.3,  -0.85, -0.65,
    -0.45, -0.85,  0.0,
    -0.3,  -0.85,  0.65,
    0.3,  -0.85, -0.65,
    0.45, -0.85,  0.0,
    0.3,  -0.85,  0.65
};


OnlinePlanner::OnlinePlanner(void)
{
    for(int i = 0; i < 6; i++)
        initialBodyPosition[i] = BASIC_BODY_POSITION[i];
    
    for(int i = 0; i < 18; i++)
        initialFeetPosition[i] = BASIC_FEET_POSITION[i];

    olgaitState = OGS_OFFLINE;
}

OnlinePlanner::~OnlinePlanner(void)
{
}

int OnlinePlanner::LoadData()
{
    return 0;
}

int OnlinePlanner::Initialize(int gaitMod)
{
    if ( olgaitState == OGS_OFFLINE && gaitMod == 1){
        olgaitState = OGS_ONLINE_WALK;
    }
    return 0;
}

int OnlinePlanner::Start(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_WALK ){
    }
    return 0;
}

int OnlinePlanner::Stop(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_WALK )
    {
    }
    return 0;
}

int OnlinePlanner::Offline()
{
    if ( olgaitState == OGS_ONLINE_WALK )
    {
        olgaitState = OGS_OFFLINE;
    }
    return 0;
}

int OnlinePlanner::GetInitialJointLength(double jointLength[])
{
    if (jointLength == nullptr)
        return -1;
    return 0;
}

int OnlinePlanner::GenerateJointTrajectory(
        double timeNow,
        Aris::RT_CONTROL::CMachineData& machineData,
        double jointLength[])
{
    if ( olgaitState == OGS_OFFLINE)
        return -1; // This function should not be called when olgaitState == OGS_OFFLINE
    
    if ( olgaitState == OGS_ONLINE_WALK)
    {
        this->CalculateEachLegPosition();
    }

    return 0;
}


int OnlinePlanner::GetForwardLegPositions(double jointLengthList[], double legTipPositions[])
{
    return 0;
}

int OnlinePlanner::CalculateEachLegPosition()
{
    for(int i = 0; i < 18; i++)
    {
        feetPosition[i] = initialFeetPosition[i];
    }
    for(int i = 0; i < 2; i++)
    {
        for( int j = 0; j < 3; j++)
        {
            feetPosition[(j * 2 + i) * 3 + 0] += legGroupPosition[(1 - i) * 3 + 0]; // all X offset
            feetPosition[(j * 2 + i) * 3 + 1] += legGroupPosition[(1 - i) * 3 + 1]; // all h offset
            feetPosition[(j * 2 + i) * 3 + 2] += legGroupPosition[(1 - i) * 3 + 2]; // all Z offset
        }
    }
    return 0;
}

