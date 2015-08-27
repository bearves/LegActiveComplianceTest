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

