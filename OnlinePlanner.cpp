#include "OnlinePlanner.h"

using namespace RobotHighLevelControl;
using namespace Model;

OnlinePlanner::OnlinePlanner(void)
{
    m_initialFlag = false;
    m_startOnlineGaitFlag = false;
    olgaitState = OGS_OFFLINE;

    for(int i = 0; i < 6; i++)
    {
        legList[i].SetID(i);
    }
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
        m_initialFlag = true;
        olgaitState = OGS_ONLINE_WALK;
    }
    else if (olgaitState == OGS_OFFLINE && gaitMod == 2){
        m_initialFlag = true;
        olgaitState = OGS_ONLINE_GOTO_START_POINT;
    }
    return 0;
}

int OnlinePlanner::Start(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_WALK ){
        m_startOnlineGaitFlag = true;
    }
    else if (olgaitState == OGS_ONLINE_GOTO_START_POINT){
        m_startOnlineGaitFlag = true;
    }
    return 0;
}

int OnlinePlanner::Stop(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_WALK )
    {
        m_impedancePlanner.Stop();
    }
    return 0;
}

int OnlinePlanner::Offline()
{
    if ( olgaitState == OGS_ONLINE_WALK || olgaitState == OGS_ONLINE_GOTO_START_POINT)
    {
        m_initialFlag = false;
        m_startOnlineGaitFlag = false;
        olgaitState = OGS_OFFLINE;
    }
    return 0;
}

int OnlinePlanner::GetInitialJointLength(double jointLength[])
{
    if (jointLength == nullptr)
        return -1;

    // CURRENT STAGE: the initial joint length is used for imdepdance planner
    m_impedancePlanner.GetInitialJointLength(jointLength);

    for( int i = 0; i < 6; i++)
    {
        rt_printf("Joint Length: %d %d %d\n", jointLength[i*3], jointLength[i*3+1], jointLength[i*3+2]);
    }
    //double initialFootTipPosition[3] = {0, 0, 0.65};
    //for(int i = 0; i < 6; i++)
    //{
        //legList[i].InverseSolutionPole(initialFootTipPosition, &jointLength[i * 3], false);
    //}

    return 0;
}

int OnlinePlanner::GenerateJointTrajectory(double timeNow, double* jointStateInput, Aris::RT_CONTROL::CForceData* forceData, double* jointStateOutput)
{
    if ( olgaitState == OGS_OFFLINE){
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointStateOutput[i] = jointStateInput[i];
        }
        return -1; // This function should not be called when olgaitState == OGS_OFFLINE
    }
    else if ( olgaitState == OGS_ONLINE_WALK)
    {
        // when the goto to start point command is received, the m_initialFlag will be set ,
        // afterwards, the trjplanner will check this flag and do the corresponding initialization
        // works.
        if (m_initialFlag == true){
            m_initialFlag = false;
            m_impedancePlanner.Initialize();
        }
        // when the start gait command is received, this flag will be set,
        // afterwards, the trjplanner will check this flag and start to move
        if (m_startOnlineGaitFlag == true)
        {
            m_startOnlineGaitFlag = false;
            m_impedancePlanner.Start(timeNow);
        }
        m_impedancePlanner.GenerateJointTrajectory(timeNow, jointStateInput, forceData, jointStateOutput);
    }
    else if ( olgaitState == OGS_ONLINE_GOTO_START_POINT)
    {
        // when the goto to start point command is received, the m_initialFlag will be set ,
        // afterwards, the trjplanner will check this flag and do the corresponding initialization
        // works, e.g. set the start and end point of the PTP trj generator
        if (m_initialFlag == true){
            m_initialFlag = false;
            GetInitialJointLength(m_destPoint);
            m_gotoPointPlanner.SetStartAndEndPoint(jointStateInput, m_destPoint);
        }
        // when the start gait command is received, this flag will be set,
        // afterwards, the trjplanner will check this flag and start to move
        if (m_startOnlineGaitFlag == true)
        {
            m_startOnlineGaitFlag = false;
            m_gotoPointPlanner.Start(timeNow);
        }
        m_gotoPointPlanner.GenerateJointTrajectory(timeNow, jointStateInput, jointStateOutput);

    }

    return 0;
}

