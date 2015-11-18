#include "OnlinePlanner.h"

using namespace RobotHighLevelControl;

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
    if ( olgaitState == OGS_OFFLINE )
    {
        switch (gaitMod)
        {
            case 1:
                olgaitState = OGS_ONLINE_WALK;
                m_initialFlag = true;
                break;
            case 2:
                olgaitState = OGS_ONLINE_GOTO_START_POINT;
                m_initialFlag = true;
                break;
            case 3:
                olgaitState = OGS_ONLINE_GOTO_SIT_POINT;
                m_initialFlag = true;
                break;
            case 4:
                olgaitState = OGS_ONLINE_GOTO_STAND_POINT;
                m_initialFlag = true;
                break;
        }
    }
    return 0;
}

int OnlinePlanner::Start(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_WALK ||
         olgaitState == OGS_ONLINE_GOTO_START_POINT ||
         olgaitState == OGS_ONLINE_GOTO_SIT_POINT ||
         olgaitState == OGS_ONLINE_GOTO_STAND_POINT)
    {
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
    if ( olgaitState == OGS_ONLINE_WALK ||
         olgaitState == OGS_ONLINE_GOTO_START_POINT ||
         olgaitState == OGS_ONLINE_GOTO_SIT_POINT ||
         olgaitState == OGS_ONLINE_GOTO_STAND_POINT)
    {
        m_initialFlag = false;
        m_startOnlineGaitFlag = false;
        olgaitState = OGS_OFFLINE;
    }
    return 0;
}

int OnlinePlanner::GetInitialJointLength(double jointLength[], ONLINE_GAIT_STATE gaitState)
{
    static double sitFootTipPosition[3] = {0, 0, 0.62};
    static double standFootTipPosition[3] = {0, 0, 0.7};
    if (jointLength == nullptr)
        return -1;

    switch (gaitState)
    {
        case OGS_ONLINE_GOTO_START_POINT:
            m_impedancePlanner.GetInitialJointLength(jointLength);
            break;
        
        case OGS_ONLINE_GOTO_SIT_POINT:
            for(int i = 0; i < 6; i++)
            {
                legList[i].InverseSolutionPole(sitFootTipPosition, &jointLength[i * 3], false);
            }
            break;

        case OGS_ONLINE_GOTO_STAND_POINT:
            for(int i = 0; i < 6; i++)
            {
                legList[i].InverseSolutionPole(standFootTipPosition, &jointLength[i * 3], false);
            }
            break;
    }

    for( int i = 0; i < 6; i++)
    {
        rt_printf("Joint Length: %d %d %d\n", jointLength[i*3], jointLength[i*3+1], jointLength[i*3+2]);
    }

    return 0;
}

int OnlinePlanner::GenerateJointTrajectory(
        double timeNow, 
        double* jointStateInput, 
        Aris::RT_CONTROL::CForceData* forceData, 
        Aris::RT_CONTROL::CIMUData& imuFdbk,
        double* jointStateOutput,
        char* controlDataForLog)
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
        m_impedancePlanner.GenerateJointTrajectory(
                timeNow, 
                m_lastHeartbeatTime,
                jointStateInput, 
                forceData, 
                imuFdbk, 
                jointStateOutput, 
                controlDataForLog);
    }
    else if ( olgaitState == OGS_ONLINE_GOTO_START_POINT ||
              olgaitState == OGS_ONLINE_GOTO_SIT_POINT ||
              olgaitState == OGS_ONLINE_GOTO_STAND_POINT)
    {
        // when the goto to start point command is received, the m_initialFlag will be set ,
        // afterwards, the trjplanner will check this flag and do the corresponding initialization
        // works, e.g. set the start and end point of the PTP trj generator
        if (m_initialFlag == true){
            m_initialFlag = false;
            GetInitialJointLength(m_destPoint, olgaitState);
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

int OnlinePlanner::SetGaitParameter(const void* paramData, int dataLength, int gaitMod)
{
    if (gaitMod == 1)
    {
        m_impedancePlanner.SetGaitParameter(paramData, dataLength);
    }
    return 0;
}

void OnlinePlanner::UpdateHearbeatTimer(double updateTime)
{
    m_lastHeartbeatTime = updateTime;
}
