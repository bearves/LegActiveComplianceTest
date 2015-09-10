#include "Planners.h"

using namespace RobotHighLevelControl;

GoToPointPlanner::GoToPointPlanner()
{
    m_state = UNREADY;
    // PTP moving should be accomplished in m_totalTime seconds
    m_totalTime = 5;
    m_timeAcc = 0.4;
    m_timeDcc = 0.4;
}

GoToPointPlanner::~GoToPointPlanner()
{
}

int GoToPointPlanner::SetStartAndEndPoint(double* currentPoint, double* destPoint)
{
    // the destination should be set when the planner has not gone moving
    if (m_state == READY || m_state == FINISHED || m_state == UNREADY)
    {
        for(int i = 0; i < 18; i++)
        {
            m_beginPointPosition[i] = currentPoint[i];
            m_destPointPosition[i] = destPoint[i];
        }
        m_state = READY;
    }
    return 0;
}

int GoToPointPlanner::Start(double timeNow)
{
    // only when the destination has been set, the online planning can be started
    if (m_state == READY)
    {
        m_state = INMOTION;
        m_timeWhenBeginToGo = timeNow;
    }
    return 0;
}

int GoToPointPlanner::GenerateJointTrajectory(double timeNow, double* currentPoint, double* jointLength)
{
    if (m_state == READY)
    {
        // Stay at the start point
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = m_beginPointPosition[i];
        }
    }
    else if (m_state == INMOTION)
    {
        // Planning to the destination
        double timeRatio = (timeNow - m_timeWhenBeginToGo)/m_totalTime;
        double posRatio = 0;
        double acc = -2 / (m_timeAcc * (m_timeAcc + m_timeDcc - 2));
        double dcc =  2 / (m_timeDcc * (m_timeAcc + m_timeDcc - 2));
        double velConstant = m_timeAcc * acc;
        timeRatio = timeRatio > 0 ? timeRatio : 0;
        if (timeRatio > 1)
        {
            // the PTP planning is finished
            m_state = FINISHED;
            timeRatio = 1;
        }

        if (timeRatio < m_timeAcc)
        {
            posRatio = 0.5 * acc * timeRatio * timeRatio;
        }
        else if (timeRatio < 1 - m_timeDcc)
        {
            posRatio = 0.5 * velConstant * m_timeAcc + velConstant * (timeRatio - m_timeAcc); 
        }
        else 
        {
            double t = (timeRatio - 1 + m_timeDcc);
            posRatio = 0.5 * velConstant * m_timeAcc + velConstant * (1 - m_timeDcc - m_timeAcc); 
            posRatio += velConstant * t + 0.5 * dcc * t * t;
        }
        for (int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = posRatio * m_destPointPosition[i] + (1 - posRatio) * m_beginPointPosition[i];
        }
        
    }
    else if (m_state == FINISHED)
    {
        // Stay at the destination 
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = m_destPointPosition[i];
        }
    }
    else if (m_state == UNREADY)
    {
        // Stay at current point
        for(int i = 0; i < AXIS_NUMBER; i++)
        {
            jointLength[i] = currentPoint[i];
        }
    }
    return 0;
}


const double ImpedancePlanner::FOOT_POS_UP_LIMIT[3]  = { Model::PI/9,  Model::PI/36, 0.76};
const double ImpedancePlanner::FOOT_POS_LOW_LIMIT[3] = {-Model::PI/9, -Model::PI/36, 0.5};
const double ImpedancePlanner::FORCE_DEADZONE[3]     = { 8, 4, 4 };

ImpedancePlanner::ImpedancePlanner()
{
    for(int i = 0; i < 6; i++)
    {
        m_legList[i].SetID(i);
    }
    for(int i = 0; i < 6; i++)
    {
        m_beginFootPos[i*3] = 0;
        m_beginFootPos[i*3 + 1] = 0;
        if ( i % 2 )
        {
            m_beginFootPos[i*3 + 2] = 0.75;
        }
        else
        {
            m_beginFootPos[i*3 + 2] = 0.75;
        }
    }
    m_state = UNREADY;
}

ImpedancePlanner::~ImpedancePlanner()
{
}

int ImpedancePlanner::Initialize()
{
    if (m_state == UNREADY || m_state == FINISHED)
    {
        m_state = READY;
        return 0;
    }
    else
    {
        return -1;
    }
}

int ImpedancePlanner::GetInitialJointLength(double* jointLength)
{
    for(int i = 0; i < 6; i++)
    {
        m_legList[i].InverseSolutionPole(&m_beginFootPos[i*3], &jointLength[i*3], false);
    }
    return 0;
}

int ImpedancePlanner::Start(double timeNow)
{
    if (m_state == READY)
    {
        m_state = INMOTION;
        m_timeWhenBeginToGo = timeNow;
    }
    return 0;
}

int ImpedancePlanner::Stop()
{
    m_state = FINISHED;
    return 0;
}

int ImpedancePlanner::GenerateJointTrajectory(
        double timeNow, 
        double* currentPoint, 
        Aris::RT_CONTROL::CForceData* forceInput, 
        double* jointLength)
{
    // When the controller has not been started, stay at the initial place
    if (m_state == UNREADY || m_state == READY)
    {
        GetInitialJointLength(m_currentTargetJointPos);
        for(int i = 0; i < 18; i++)
        {
            m_currentAdjustedJointPos[i] = m_currentTargetJointPos[i];
            m_currentAdjustedFootPos[i]  = m_beginFootPos[i];
            m_currentAdjustedFootVel[i]  = 0;
            m_currentOffset[i]    = 0;
            m_currentOffsetdot[i] = 0;
            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
            jointLength[i] = m_currentAdjustedJointPos[i];
        }
    }
    else if (m_state == INMOTION)
    {
        // The legs are asked to stay at the initial position in current stage
        for( int i = 0; i < 18; i++)
        {
            m_currentTargetFootPos[i] = m_beginFootPos[i];
            m_currentTargetFootVel[i] = 0; 
            m_forceDesire[i] = 0;
        }
        // Do the force transformation
        for( int i = 0; i < 6; i++)
        {
            for( int j = 0; j < 3; j++)
            {
                m_forceRaw[i*3 + j] = forceInput[i].forceValues[j]/1000.0;
            }
            ForceTransform(&m_forceRaw[i*3], &m_currentAdjustedFootPos[i*3], &m_forceTransfromed[i*3]);
            DeadZone(&m_forceTransfromed[i*3]);
        }

        
        // Do the impedance adjustment
        for( int i = 0; i < 6; i++)
        {
            ImpedanceControl(&m_forceTransfromed[i*3], &m_forceDesire[i*3], 
                             &m_lastOffset[i*3], &m_lastOffsetdot[i*3],
                             &m_currentOffset[i*3], &m_currentOffsetdot[i*3], i);

            // Only test the length direction of the legs 
            for (int j = 0; j < 2; ++j) {
                m_currentOffsetdot[i*3+j] = 0;
                m_currentOffset[i*3+j] = 0;
            }
        }

        for( int i = 0; i < 18; i++)
        {
            m_currentAdjustedFootPos[i] = m_currentTargetFootPos[i] - m_currentOffset[i];
            m_currentAdjustedFootVel[i] = m_currentTargetFootVel[i] - m_currentOffsetdot[i];
            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
        }
        //
        // Get the joint length
        for(int i = 0; i < 6; i++)
        {
            SaturateProcess(&m_currentAdjustedFootPos[i*3]);
            m_legList[i].InverseSolutionPole(&m_currentAdjustedFootPos[i*3], &m_currentAdjustedJointPos[i*3], false);
        }

        if (fmod(timeNow, 0.5) < 1.1e-3)
        {
            for (int i = 0; i < 6; ++i)
            {
                rt_printf("TF: ");
                for (int j = 0; j < 3; ++j)
                {
                    rt_printf("%7.6lf  ", m_currentOffset[i*3+j]);
                }
                rt_printf("\n");
            }
            rt_printf("\n");
        }
        // Output
        for(int i = 0; i < 18; i++)
        {
            jointLength[i] = m_currentAdjustedJointPos[i];
        }
    }
    // When the controller has been stopped, the leg should held its current place
    else if (m_state == FINISHED)
    {
        // since the m_currentAdjustedJointPos will be no longer updated,
        // the leg will be held at that place
        for(int i = 0; i < 18; i++)
        {
            m_currentOffset[i] = 0;
            m_currentOffsetdot[i] = 0;
            m_lastOffset[i] = m_currentOffset[i];
            m_lastOffsetdot[i] = m_currentOffsetdot[i];
            jointLength[i] = m_currentAdjustedJointPos[i];
        }
    }
    return 0;
}

int ImpedancePlanner::ForceTransform(double* forceRaw, double* legPositionEstimated, double* forceTransformed)
{
    using namespace Model;
    // Firstly, the x and z direction on the actual force sensor are opposite to the matlab simulink models
    double fx = -forceRaw[0];
    double fy = forceRaw[1];
    double fz = -forceRaw[2];
    double l = legPositionEstimated[2];
    double a0 = (LG1 + LG3)/1000.0;
    double b0 = LG2 / 1000.0;
    // Secondly, we need to rotate the z direction towards the hip joint
    double tmp = (b0*b0-a0*a0-l*l)/(-2 * a0 * l);
    double alpha = 0;
    if ( fabs(tmp) < 1.0)
        alpha = acos(tmp);
    double sa = std::sin(alpha);
    double ca = std::cos(alpha);
    forceTransformed[2] = ca * fz - sa * fx;
    forceTransformed[0] = (ca * fx + sa * fz) * l;
    forceTransformed[1] = fy * l;

    return 0;
}

int ImpedancePlanner::ImpedanceControl(double* forceInput, double* forceDesire,
        double* lastOffset, double* lastOffsetdot,
        double* currentOffset, double* currentOffsetdot, int legID)
{
    double K_ac[3] = {1e8, 1e8, 1.6e4};
    double B_ac[3] = {1e5, 1e5, 3000};
    double M_ac[3] = {100, 100, 120};
    double deltaF[3]; 

    if (legID == Model::Leg::LEG_ID_MB || legID == Model::Leg::LEG_ID_MF)
    {
        K_ac[2] *= 2;
        B_ac[2] *= 2;
    }
    
    for (int i = 0; i < 3; ++i) 
    {
        deltaF[i] = forceInput[i] - forceDesire[i];
    }
    double th = 0.001;
    double xddot[3];
    for (int i = 0; i < 3; ++i) {
        xddot[i] = (deltaF[i] - K_ac[i] * lastOffset[i] - B_ac[i] * lastOffsetdot[i]) / M_ac[i];
        currentOffsetdot[i] = lastOffsetdot[i] + xddot[i] * th;
        currentOffset[i] = lastOffset[i] + currentOffsetdot[i] * th;
    }

    return 0;
}

int ImpedancePlanner::SaturateProcess(double* adjustedFootPos)
{
    for (int i = 0; i < 3; ++i) {
        if (adjustedFootPos[i] < FOOT_POS_LOW_LIMIT[i])
        {
            adjustedFootPos[i] = FOOT_POS_LOW_LIMIT[i];
        }
        else if (adjustedFootPos[i] > FOOT_POS_UP_LIMIT[i])
        {
            adjustedFootPos[i] = FOOT_POS_UP_LIMIT[i];
        }
    }
    return 0;
}

int ImpedancePlanner::DeadZone(double* force)
{
    for (int i = 0; i < 3; ++i) {
        if(fabs(force[i]) < FORCE_DEADZONE[i])
            force[i] = 0;
    }
    return 0;
}
